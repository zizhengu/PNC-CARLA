#pragma once

#include <Eigen/Dense>
#include <vector>

// 定义 UKF 所需的常量
const int STATE_DIM = 7; // 状态维度 [x, y, vx, vy, ax, ay, heading]
const int MEAS_DIM = 7;  // 测量维度 [x, y, vx, vy, ax, ay, heading]
const int SIGMA_POINTS = 2 * STATE_DIM + 1;

class UnscentedKalmanFilter
{
private:
    Eigen::VectorXd state;             // 状态向量
    Eigen::MatrixXd covariance;        // 状态协方差矩阵
    double alpha;                      // UKF 参数
    double beta;                       // UKF参数
    double kappa;                      // UKF参数
    double lambda;                     // UKF参数
    Eigen::MatrixXd sigma_points;      // Sigma点
    Eigen::VectorXd weights_m;         // 用于均值的权重
    Eigen::VectorXd weights_c;         // 用于协方差的权重
    Eigen::MatrixXd process_noise;     // 过程噪声协方差矩阵
    Eigen::MatrixXd measurement_noise; // 测量噪声协方差矩阵

    // 生成 Sigma 点
    void generateSigmaPoints();

    // 预测 Sigma 点
    Eigen::MatrixXd predictSigmaPoints(const Eigen::MatrixXd &sigma_points, double dt);

    // 从 Sigma 点计算均值和协方差
    void computeMeanAndCovariance(const Eigen::MatrixXd &sigma_points, Eigen::VectorXd &mean, Eigen::MatrixXd &cov);

    // 测量模型
    Eigen::MatrixXd measurementModel(const Eigen::MatrixXd &sigma_points);

public:
    UnscentedKalmanFilter();

    // 初始化滤波器
    void initialize(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance);

    // 预测步骤
    void predict(double dt);

    // 更新步骤
    void update(const Eigen::VectorXd &measurement);

    // 预测未来位置
    std::vector<Eigen::VectorXd> predictFuturePositions(int steps, double dt);
};