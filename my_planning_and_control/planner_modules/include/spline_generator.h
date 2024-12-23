#pragma once

#include <vector>
#include "bezier.h"
#include "basics.h"

template <int N_DIM>
struct SpatioTemporalSemanticCubeNd
{
    decimal_t t_lb, t_ub;
    std::array<decimal_t, N_DIM> p_lb, p_ub;
    std::array<decimal_t, N_DIM> v_lb, v_ub;
    std::array<decimal_t, N_DIM> a_lb, a_ub;

    SpatioTemporalSemanticCubeNd() { FillDefaultBounds(); }

    void FillDefaultBounds()
    {
        // ~ for optimization, infinity bound will cause numerical
        // ~ unstable. So we put some loose bounds by default
        t_lb = 0.0;
        t_ub = 1.0;

        const decimal_t default_pos_lb = -100;
        const decimal_t default_pos_ub = 100;
        const decimal_t default_vel_lb = -20.0;
        const decimal_t default_vel_ub = 20.0;
        const decimal_t default_acc_lb = -20.0;
        const decimal_t default_acc_ub = 20.0;

        p_lb.fill(default_pos_lb);
        v_lb.fill(default_vel_lb);
        a_lb.fill(default_acc_lb);

        p_ub.fill(default_pos_ub);
        v_ub.fill(default_vel_ub);
        a_ub.fill(default_acc_ub);
    }
};

template <int N_DEG, int N_DIM>
class SplineGenerator
{
public:
    typedef BezierSpline<N_DEG, N_DIM> BezierSplineType;

    static long long fac(int n)
    {
        if (n == 0)
            return 1;
        if (n == 1)
            return 1;
        if (n == 2)
            return 2;
        if (n == 3)
            return 6;
        if (n == 4)
            return 24;
        if (n == 5)
            return 120;

        long long ans = 1;
        for (int i = 1; i <= n; i++)
            ans *= i;
        return ans;
    }

    static long long nchoosek(int n, int k) { return fac(n) / fac(k) / fac(n - k); }

    static ErrorType GetBezierSplineUsingCorridor(
        const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>> &cubes,
        const vec_E<Vecf<N_DIM>> &start_constraints,
        const vec_E<Vecf<N_DIM>> &end_constraints,
        const std::vector<decimal_t> &ref_stamps,
        const vec_E<Vecf<N_DIM>> &ref_points, const std::vector<double> &weight_proximity,
        BezierSplineType *bezier_spline);

}; // class spline generator
