#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>

namespace kalman
{
    class KalmanFilter
    {
    public:
        // 构造函数
        KalmanFilter() : dim_x(3), dim_u(5), t(0.1)
        {
            A.resize(dim_x, dim_x);
            A << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;

            B.resize(dim_x, dim_u);
            B << t, 0, 0.5 * t * t, 0, 0,
                0, t, 0, 0.5 * t * t, 0,
                0, 0, 0, 0, t;

            H.resize(dim_x, dim_x);
            H << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;

            Q.resize(dim_x, dim_x);
            Q << 0.05, 0, 0,
                0, 0.05, 0,
                0, 0, 0.05;

            R.resize(dim_x, dim_x);
            R << 1.0, 0, 0,
                0, 1.0, 0,
                0, 0, 1.0;

            P.resize(dim_x, dim_x);
            P << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;

            z.resize(dim_x);
            x_l_k.resize(dim_x);
            x_p_k.resize(dim_x);
            u.resize(dim_u);
        }

        Eigen::VectorXd x_p_k; // 先验预测的状态向量
        Eigen::VectorXd x_l_k; // 后验上一时刻的状态向量
        // 预测方法
        // 控制量和观测值
        Eigen::VectorXd predict(Eigen::VectorXd &u, Eigen::VectorXd &xt);
        // 更新方法（示例）
        Eigen::VectorXd update();

        Eigen::MatrixXd GetCovariance() const;

    private:
        int dim_x; // 状态向量的维度
        int dim_u; // 控制输入向量的维度
        double t;  // 时间步长

        Eigen::MatrixXd A; // 状态转移矩阵
        Eigen::MatrixXd B; // 控制输入矩阵
        Eigen::MatrixXd Q; // 过程噪声协方差矩阵
        Eigen::MatrixXd P; // 误差协方差矩阵
        Eigen::MatrixXd H; // 测量矩阵
        Eigen::MatrixXd R; // 测量噪声协方差矩阵

        Eigen::VectorXd z;
        Eigen::VectorXd u; // 控制输入向量
    };
}

#endif