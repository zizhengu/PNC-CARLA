#include "KalmanFilter.h"

// 预测方法
Eigen::VectorXd kalman::KalmanFilter::predict(Eigen::VectorXd &u, Eigen::VectorXd &xt)
{
    // double delta_t = t;
    // x_p_k = A * x_l_k + B * u + noise;
    x_p_k = A * x_l_k + B * u;
    P = A * P * A.transpose() + Q;
    z = H * xt;
    return x_p_k;
}

Eigen::VectorXd kalman::KalmanFilter::update()
{
    // 更新步骤
    Eigen::VectorXd y = z - H * x_p_k;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x_l_k = x_p_k + K * y;
    P = (Eigen::MatrixXd::Identity(dim_x, dim_x) - K * H) * P;
    return x_l_k;
}

// 获取不确定性矩阵 P
Eigen::MatrixXd kalman::KalmanFilter::GetCovariance() const
{
    return P;
}