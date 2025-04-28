// KalmanFilter.cpp
#include "KalmanFilterCTRV.h"
#include <cmath>

KalmanFilterCTRV::KalmanFilterCTRV()
{
    // 初始化状态向量
    x_ = Eigen::VectorXd(5);
    x_ << 0, 0, 0, 0, 0;

    // 初始化协方差矩阵
    P_ = Eigen::MatrixXd(5, 5);
    P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    // 过程噪声协方差矩阵
    Q_ = Eigen::MatrixXd(5, 5);
    Q_ << 0.1, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0,
        0, 0, 0.1, 0, 0,
        0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0.1;

    // 观测矩阵
    H_ = Eigen::MatrixXd(2, 5);
    H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

    // 观测噪声协方差矩阵
    R_ = Eigen::MatrixXd(2, 2);
    R_ << 0.1, 0,
        0, 0.1;
}
void KalmanFilterCTRV::initializeState(double px, double py, double v, double yaw, double yawd)
{
    x_ << px, py, v, yaw, yawd;
}
void KalmanFilterCTRV::predict(double dt)
{
    double px = x_(0);
    double py = x_(1);
    double v = x_(2);
    double yaw = x_(3);
    double yawd = x_(4);

    // 状态转移矩阵
    Eigen::MatrixXd F(5, 5);
    if (std::fabs(yawd) > 0.001)
    {
        F << 1, 0, (std::sin(yaw + yawd * dt) - std::sin(yaw)) / yawd, 0, (-v * std::cos(yaw + yawd * dt) + v * std::cos(yaw)) / yawd,
            0, 1, (-std::cos(yaw + yawd * dt) + std::cos(yaw)) / yawd, 0, (-v * std::sin(yaw + yawd * dt) + v * std::sin(yaw)) / yawd,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, dt,
            0, 0, 0, 0, 1;
    }
    else
    {
        F << 1, 0, dt * std::cos(yaw), 0, -0.5 * dt * dt * std::sin(yaw),
            0, 1, dt * std::sin(yaw), 0, 0.5 * dt * dt * std::cos(yaw),
            0, 0, 1, 0, 0,
            0, 0, 0, 1, dt,
            0, 0, 0, 0, 1;
    }

    // 预测状态
    x_ = F * x_;

    // 预测协方差矩阵
    P_ = F * P_ * F.transpose() + Q_;
}

void KalmanFilterCTRV::update(const Eigen::VectorXd &z)
{
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // 更新状态
    x_ = x_ + K * y;

    // 更新协方差矩阵
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilterCTRV::getState() const
{
    return x_;
}

Eigen::MatrixXd KalmanFilterCTRV::getCovariance() const
{
    return P_;
}