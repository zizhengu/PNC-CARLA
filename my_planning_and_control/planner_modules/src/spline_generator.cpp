#include "ooqp_interface.h"
#include "spline_generator.h"
#include "rclcpp/rclcpp.hpp"

auto GetBezierSpline_LOG = rclcpp::get_logger("GetBezierSpline");

template <int N_DEG, int N_DIM> // 5，2
ErrorType SplineGenerator<N_DEG, N_DIM>::GetBezierSplineUsingCorridor(
    const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>> &cubes,
    const vec_E<Vecf<N_DIM>> &start_constraints,
    const vec_E<Vecf<N_DIM>> &end_constraints,
    const std::vector<decimal_t> &ref_stamps,
    const vec_E<Vecf<N_DIM>> &ref_points, const std::vector<double> &weight_proximity,
    BezierSplineType *bezier_spline)
{

    int num_segments = static_cast<int>(cubes.size()); // num_segments = 4
    int num_order = N_DEG + 1;                         // num_order = 6
    int derivative_degree = 3;
    // RCLCPP_INFO(GetBezierSpline_LOG, "stack objective");
    // ~ Stage I: stack objective
    /**
     * 目标一：满足贝塞尔曲线性质的jerk 平方和最小
     * 目标二：路径与参考点误差最小
     */
    int total_num_vals = N_DIM * num_segments * num_order; // 2 * 4 * 6
    // RCLCPP_INFO(GetBezierSpline_LOG, "N_DIM: %i, num_segments: %i, num_order %i", N_DIM, num_segments, num_order);
    // 创建一个大小为48*48 的稀疏矩阵
    Eigen::SparseMatrix<double, Eigen::RowMajor> Q(total_num_vals,
                                                   total_num_vals);
    // Eigen::VectorXi::Constant(...) 创建一个大小为48，的向量，所有值为 6。
    // RCLCPP_INFO(GetBezierSpline_LOG, "Q.reserve");
    Q.reserve(
        Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
    {
        // 因为控制点满足贝塞尔曲线控制点的性质，所以才有这个海森矩阵
        // RCLCPP_INFO(GetBezierSpline_LOG, "hessian = ");
        MatNf<N_DEG + 1> hessian =
            BezierUtils<N_DEG>::GetBezierHessianMat(derivative_degree);
        int idx, idy;
        decimal_t val;
        // RCLCPP_INFO(GetBezierSpline_LOG, "Q.insert");
        for (int n = 0; n < num_segments; n++)
        {
            decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
            for (int d = 0; d < N_DIM; d++)
            {
                for (int j = 0; j < num_order; j++)
                {
                    for (int k = 0; k < num_order; k++)
                    {
                        idx = d * num_segments * num_order + n * num_order + j;
                        idy = d * num_segments * num_order + n * num_order + k;
                        val = hessian(j, k) / pow(duration, 2 * derivative_degree - 3);
                        Q.insert(idx, idy) = val;
                    }
                }
            }
        }
    }

    Eigen::VectorXd c;
    c.resize(total_num_vals);
    c.setZero();

    Eigen::SparseMatrix<double, Eigen::RowMajor> P(total_num_vals,
                                                   total_num_vals);
    // RCLCPP_INFO(GetBezierSpline_LOG, "P.reserve");
    P.reserve(
        Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));

    // RCLCPP_INFO(GetBezierSpline_LOG, "if (!ref_stamps.empty())");
    // * Only position difference is considered
    if (!ref_stamps.empty())
    {
        int idx, idy;
        int num_ref_samples = ref_stamps.size(); // 14
        for (int i = 0; i < num_ref_samples; i++)
        {
            if (ref_stamps[i] < cubes[0].t_lb ||
                ref_stamps[i] > cubes[num_segments - 1].t_ub)
                continue;

            int n;
            for (n = 0; n < num_segments; n++)
            {
                if (cubes[n].t_ub > ref_stamps[i])
                {
                    break;
                }
            }
            n = std::min(num_segments - 1, n);
            decimal_t s = cubes[n].t_ub - cubes[n].t_lb;
            decimal_t t = ref_stamps[i] - cubes[n].t_lb;
            for (int d = 0; d < N_DIM; d++)
            {
                for (int j = 0; j < num_order; j++)
                {
                    idx = d * num_segments * num_order + n * num_order + j;
                    c[idx] += -2 * ref_points[i][d] * s * nchoosek(N_DEG, j) *
                              pow(t / s, j) * pow(1 - t / s, N_DEG - j);
                    for (int k = 0; k < num_order; k++)
                    {
                        idy = d * num_segments * num_order + n * num_order + k;
                        P.coeffRef(idx, idy) += s * s * nchoosek(N_DEG, j) *
                                                nchoosek(N_DEG, k) * pow(t / s, j + k) *
                                                pow(1 - t / s, 2 * N_DEG - j - k);
                    }
                }
            }
        }
    }
    // std::cout << "Q = \n" << Q << "\n" << std::endl;
    // std::cout << "P = \n" << P << "\n" << std::endl;
    // std::cout << "c = \n" << c << "\n" << std::endl;

    P = P * weight_proximity[0];
    c = c * weight_proximity[1];

    Q = 2 * (Q + P); // 0.5 * x' * Q * x

    // RCLCPP_INFO(GetBezierSpline_LOG, "stack equality constraints");
    // ~ Stage II: stack equality constraints
    /**
     * 等式约束一： 两个维度上的连续性约束 continuity up to jerk
     * 等式约束二： 起始点约束
     * 等式约束三： 终点约束
     */
    int num_continuity = 3;                 // continuity up to jerk
    int num_connections = num_segments - 1; // 3
    // printf("num conenctions: %d.\n", num_connections);
    int num_continuity_constraints = N_DIM * num_connections * num_continuity; // 2 * 3 * 3 = 18
    int num_start_eq_constraints =
        static_cast<int>(start_constraints.size()) * N_DIM; // 3 * 2 = 6
    int num_end_eq_constraints = static_cast<int>(end_constraints.size()) * N_DIM;
    // int total_num_eq_constraints = num_continuity_constraints +
    //                                num_start_eq_constraints +
    //                                num_end_eq_constraints; // 18 + 6 + 6 = 30
    int total_num_eq_constraints = num_continuity_constraints;
    Eigen::SparseMatrix<double, Eigen::RowMajor> A(
        total_num_eq_constraints, N_DIM * num_segments * num_order); // 30 * 48
    A.reserve(Eigen::VectorXi::Constant(total_num_eq_constraints, 2 * num_order));

    Eigen::VectorXd b;
    b.resize(total_num_eq_constraints);
    b.setZero();

    int idx, idy;
    decimal_t val;
    {
        // RCLCPP_INFO(GetBezierSpline_LOG, "~ continuity constraints");
        // ~ continuity constraints
        for (int n = 0; n < num_connections; n++)
        {
            decimal_t duration_l = cubes[n].t_ub - cubes[n].t_lb;
            decimal_t duration_r = cubes[n + 1].t_ub - cubes[n + 1].t_lb;
            for (int c = 0; c < num_continuity; c++)
            {
                decimal_t scale_l = pow(duration_l, 1 - c);
                decimal_t scale_r = pow(duration_r, 1 - c);
                for (int d = 0; d < N_DIM; d++)
                {
                    idx = d * num_connections * num_continuity + n * num_continuity + c;
                    if (c == 0)
                    {
                        // ~ position end
                        idy = d * num_segments * num_order + n * num_order + N_DEG;
                        val = 1.0 * scale_l;
                        A.insert(idx, idy) = val;
                        // ~ position begin
                        idy = d * num_segments * num_order + (n + 1) * num_order + 0;
                        val = 1.0 * scale_r;
                        A.insert(idx, idy) = -val;
                    }
                    else if (c == 1)
                    {
                        // ~ velocity end
                        idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                        val = -1.0 * scale_l;
                        A.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + N_DEG;
                        val = 1.0 * scale_l;
                        A.insert(idx, idy) = val;
                        // ~ velocity begin
                        idy = d * num_segments * num_order + (n + 1) * num_order;
                        val = -1.0 * scale_r;
                        A.insert(idx, idy) = -val;
                        idy = d * num_segments * num_order + (n + 1) * num_order + 1;
                        val = 1.0 * scale_r;
                        A.insert(idx, idy) = -val;
                    }
                    else if (c == 2)
                    {
                        // ~ acceleration end
                        idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
                        val = 1.0 * scale_l;
                        A.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                        val = -2.0 * scale_l;
                        A.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + N_DEG;
                        val = 1.0 * scale_l;
                        A.insert(idx, idy) = val;
                        // ~ acceleration begin
                        idy = d * num_segments * num_order + (n + 1) * num_order;
                        val = 1.0 * scale_r;
                        A.insert(idx, idy) = -val;
                        idy = d * num_segments * num_order + (n + 1) * num_order + 1;
                        val = -2.0 * scale_r;
                        A.insert(idx, idy) = -val;
                        idy = d * num_segments * num_order + (n + 1) * num_order + 2;
                        val = 1.0 * scale_r;
                        A.insert(idx, idy) = -val;
                    }
                    else if (c == 3)
                    {
                        // ~ jerk end
                        idy = d * num_segments * num_order + n * num_order + N_DEG - 3;
                        val = -1.0 * scale_l;
                        A.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
                        val = 3.0 * scale_l;
                        A.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                        val = -3.0 * scale_l;
                        A.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + N_DEG;
                        val = 1.0 * scale_l;
                        A.insert(idx, idy) = val;
                        // ~ jerk begin
                        idy = d * num_segments * num_order + (n + 1) * num_order;
                        val = -1.0 * scale_r;
                        A.insert(idx, idy) = -val;
                        idy = d * num_segments * num_order + (n + 1) * num_order + 1;
                        val = 3.0 * scale_r;
                        A.insert(idx, idy) = -val;
                        idy = d * num_segments * num_order + (n + 1) * num_order + 2;
                        val = -3.0 * scale_r;
                        A.insert(idx, idy) = -val;
                        idy = d * num_segments * num_order + (n + 1) * num_order + 3;
                        val = 1.0 * scale_r;
                        A.insert(idx, idy) = -val;
                    }
                }
            }
        }
        // RCLCPP_INFO(GetBezierSpline_LOG, "~ start state constraints");
        // ~ start state constraints
        // {
        //     int num_order_constraint_start =
        //         static_cast<int>(start_constraints.size());
        //     decimal_t duration = cubes[0].t_ub - cubes[0].t_lb;
        //     decimal_t scale;
        //     int n = 0;
        //     for (int j = 0; j < num_order_constraint_start; j++)
        //     {
        //         scale = pow(duration, 1 - j);
        //         for (int d = 0; d < N_DIM; d++)
        //         {
        //             idx = num_continuity_constraints + d * num_order_constraint_start + j;
        //             if (j == 0)
        //             {
        //                 idy = d * num_segments * num_order + n * num_order + 0;
        //                 val = 1.0 * scale;
        //                 A.insert(idx, idy) = val;

        //                 b[idx] = start_constraints[j][d];
        //                 // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
        //             }
        //             else if (j == 1)
        //             {
        //                 idy = d * num_segments * num_order + n * num_order + 0;
        //                 val = -1.0 * N_DEG * scale;
        //                 A.insert(idx, idy) = val;
        //                 idy = d * num_segments * num_order + n * num_order + 1;
        //                 val = 1.0 * N_DEG * scale;
        //                 A.insert(idx, idy) = val;

        //                 b[idx] = start_constraints[j][d];
        //                 // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
        //             }
        //             else if (j == 2)
        //             {
        //                 idy = d * num_segments * num_order + n * num_order + 0;
        //                 val = 1.0 * N_DEG * (N_DEG - 1) * scale;
        //                 A.insert(idx, idy) = val;

        //                 idy = d * num_segments * num_order + n * num_order + 1;
        //                 val = -2.0 * N_DEG * (N_DEG - 1) * scale;
        //                 A.insert(idx, idy) = val;

        //                 idy = d * num_segments * num_order + n * num_order + 2;
        //                 val = 1.0 * N_DEG * (N_DEG - 1) * scale;
        //                 A.insert(idx, idy) = val;

        //                 b[idx] = start_constraints[j][d];
        //                 // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
        //             }
        //         }
        //     }
        // }
        // // ~ end state constraints
        // {
        //     int num_order_constraint_end = static_cast<int>(end_constraints.size());
        //     decimal_t duration =
        //         cubes[num_segments - 1].t_ub - cubes[num_segments - 1].t_lb;
        //     decimal_t scale;
        //     int n = num_segments - 1;
        //     int accu_eq_cons_idx =
        //         num_continuity_constraints + num_start_eq_constraints;
        //     for (int j = 0; j < num_order_constraint_end; j++)
        //     {
        //         scale = pow(duration, 1 - j);
        //         for (int d = 0; d < N_DIM; d++)
        //         {
        //             // if (j == 0 && d == 0) continue;
        //             idx = accu_eq_cons_idx++;
        //             if (j == 0)
        //             {
        //                 idy = d * num_segments * num_order + n * num_order + N_DEG;
        //                 val = 1.0 * scale;
        //                 A.insert(idx, idy) = val;
        //                 b[idx] = end_constraints[j][d];
        //             }
        //             else if (j == 1)
        //             {
        //                 idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
        //                 val = -1.0 * N_DEG * scale;
        //                 A.insert(idx, idy) = val;
        //                 idy = d * num_segments * num_order + n * num_order + N_DEG;
        //                 val = 1.0 * N_DEG * scale;
        //                 A.insert(idx, idy) = val;
        //                 b[idx] = end_constraints[j][d];
        //             }
        //             else if (j == 2)
        //             {
        //                 idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
        //                 val = 1.0 * N_DEG * (N_DEG - 1) * scale;
        //                 A.insert(idx, idy) = val;

        //                 idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
        //                 val = -2.0 * N_DEG * (N_DEG - 1) * scale;
        //                 A.insert(idx, idy) = val;

        //                 idy = d * num_segments * num_order + n * num_order + N_DEG;
        //                 val = 1.0 * N_DEG * (N_DEG - 1) * scale;
        //                 A.insert(idx, idy) = val;
        //                 b[idx] = end_constraints[j][d];
        //             }
        //         }
        //     }
        // }
    }

    // RCLCPP_INFO(GetBezierSpline_LOG, "Stage III: stack inequality constraints");
    // ~ Stage III: stack inequality constraints
    int total_num_ineq = 0;
    for (int i = 0; i < num_segments; i++)
    {
        total_num_ineq += (static_cast<int>(cubes[i].p_ub.size())) * num_order;
        total_num_ineq +=
            (static_cast<int>(cubes[i].v_ub.size())) * (num_order - 1);
        total_num_ineq +=
            (static_cast<int>(cubes[i].a_ub.size())) * (num_order - 2);
    }
    // Eigen::SparseMatrix<double, Eigen::RowMajor> C;
    Eigen::VectorXd lbd;
    Eigen::VectorXd ubd;
    Eigen::SparseMatrix<double, Eigen::RowMajor> C(total_num_ineq,
                                                   total_num_vals);
    C.reserve(Eigen::VectorXi::Constant(total_num_ineq, 3));
    lbd.setZero(total_num_ineq);
    ubd.setZero(total_num_ineq);
    {
        int accu_num_ineq = 0;
        for (int n = 0; n < num_segments; n++)
        {
            decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
            decimal_t scale;
            for (int d = 0; d < N_DIM; d++)
            {
                // ~ enforce position bounds
                scale = pow(duration, 1 - 0);
                for (int j = 0; j < num_order; j++)
                {
                    idx = accu_num_ineq;
                    idy = d * num_segments * num_order + n * num_order + j;
                    val = scale;
                    C.insert(idx, idy) = val;
                    lbd[idx] = cubes[n].p_lb[d];
                    ubd[idx] = cubes[n].p_ub[d];
                    accu_num_ineq++;
                }
                // ~ enforce velocity bounds
                scale = pow(duration, 1 - 1);
                for (int j = 0; j < num_order - 1; j++)
                {
                    idx = accu_num_ineq;
                    idy = d * num_segments * num_order + n * num_order + j;
                    val = -N_DEG * scale;
                    C.insert(idx, idy) = val;
                    idy = d * num_segments * num_order + n * num_order + (j + 1);
                    val = N_DEG * scale;
                    C.insert(idx, idy) = val;
                    lbd[idx] = cubes[n].v_lb[d];
                    ubd[idx] = cubes[n].v_ub[d];
                    accu_num_ineq++;
                }
                // ~ enforce acceleration bounds
                scale = pow(duration, 1 - 2);
                for (int j = 0; j < num_order - 2; j++)
                {
                    idx = accu_num_ineq;
                    idy = d * num_segments * num_order + n * num_order + j;
                    val = N_DEG * (N_DEG - 1) * scale;
                    C.insert(idx, idy) = val;
                    idy = d * num_segments * num_order + n * num_order + (j + 1);
                    val = -2.0 * N_DEG * (N_DEG - 1) * scale;
                    C.insert(idx, idy) = val;
                    idy = d * num_segments * num_order + n * num_order + (j + 2);
                    val = N_DEG * (N_DEG - 1) * scale;
                    C.insert(idx, idy) = val;
                    lbd[idx] = cubes[n].a_lb[d];
                    ubd[idx] = cubes[n].a_ub[d];
                    accu_num_ineq++;
                }
            }
        }
    }
    // RCLCPP_INFO(GetBezierSpline_LOG, "dummy constraints");
    // dummy constraints
    Eigen::VectorXd u = std::numeric_limits<double>::max() *
                        Eigen::VectorXd::Ones(total_num_vals);
    Eigen::VectorXd l = (-u.array()).matrix();

    // RCLCPP_INFO(GetBezierSpline_LOG, "Stage IV: solve");
    // ~ Stage IV: solve
    Eigen::VectorXd x;
    x.setZero(total_num_vals);
    if (!OoQpItf::solve(Q, c, A, b, C, lbd, ubd, l, u, x, true, false))
    {
        printf("[GetBezierSplineUsingCorridor]Solver error.\n");
        return kWrongStatus;
    }

    // RCLCPP_INFO(GetBezierSpline_LOG, "Stage V: set back to bezier struct");
    // ~ Stage V: set back to bezier struct
    std::vector<decimal_t> vec_domain;
    vec_domain.push_back(cubes.front().t_lb);
    for (int n = 0; n < num_segments; n++)
    {
        vec_domain.push_back(cubes[n].t_ub);
    }
    bezier_spline->set_vec_domain(vec_domain);
    for (int n = 0; n < num_segments; n++)
    {
        for (int j = 0; j < num_order; j++)
        {
            Vecf<N_DIM> coeff;
            for (int d = 0; d < N_DIM; d++)
                coeff[d] = x[d * num_segments * num_order + n * num_order + j];
            bezier_spline->set_coeff(n, j, coeff);
        }
    }

    return kSuccess;
}

template class SplineGenerator<5, 2>;
