// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilterCTRV
{
public:
    KalmanFilterCTRV();
    void predict(double dt);
    void update(const Eigen::VectorXd &z);
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    void initializeState(double px, double py, double v, double yaw, double yawd);


private:
    Eigen::VectorXd x_; // 状态向量
    Eigen::MatrixXd P_; // 协方差矩阵
    Eigen::MatrixXd Q_; // 过程噪声协方差矩阵
    Eigen::MatrixXd H_; // 观测矩阵
    Eigen::MatrixXd R_; // 观测噪声协方差矩阵
};

#endif