#include "UKF.h"
#include <Eigen/Dense>
#include <omp.h>
// 无损卡尔曼滤波器类的实现

// 生成 Sigma 点
void UnscentedKalmanFilter::generateSigmaPoints()
{
    Eigen::MatrixXd L = covariance.llt().matrixL();
    sigma_points.col(0) = state;
    for (int i = 0; i < STATE_DIM; ++i)
    {
        sigma_points.col(i + 1) = state + std::sqrt(STATE_DIM + lambda) * L.col(i);
        sigma_points.col(i + 1 + STATE_DIM) = state - std::sqrt(STATE_DIM + lambda) * L.col(i);
    }
}

// 预测 Sigma 点
Eigen::MatrixXd UnscentedKalmanFilter::predictSigmaPoints(const Eigen::MatrixXd &sigma_points, double dt)
{
    Eigen::MatrixXd predicted_sigma_points(STATE_DIM, SIGMA_POINTS);
    for (int i = 0; i < SIGMA_POINTS; ++i)
    {
        Eigen::VectorXd point = sigma_points.col(i);
        // 更新预测公式以包含加速度
        point(0) += point(2) * dt + 0.5 * point(4) * dt * dt;
        point(1) += point(3) * dt + 0.5 * point(5) * dt * dt;
        predicted_sigma_points.col(i) = point;
    }
    return predicted_sigma_points;
}

// 从 Sigma 点计算均值和协方差
void UnscentedKalmanFilter::computeMeanAndCovariance(const Eigen::MatrixXd &sigma_points, Eigen::VectorXd &mean, Eigen::MatrixXd &cov)
{
    mean = Eigen::VectorXd::Zero(sigma_points.rows());
    for (int i = 0; i < SIGMA_POINTS; ++i)
    {
        mean += weights_m(i) * sigma_points.col(i);
    }
    cov = Eigen::MatrixXd::Zero(sigma_points.rows(), sigma_points.rows());
    for (int i = 0; i < SIGMA_POINTS; ++i)
    {
        Eigen::VectorXd diff = sigma_points.col(i) - mean;
        cov += weights_c(i) * diff * diff.transpose();
    }
}

// 测量模型
Eigen::MatrixXd UnscentedKalmanFilter::measurementModel(const Eigen::MatrixXd &sigma_points)
{
    Eigen::MatrixXd measurement_sigma_points(MEAS_DIM, SIGMA_POINTS);
    for (int i = 0; i < SIGMA_POINTS; ++i)
    {
        measurement_sigma_points.col(i) = sigma_points.col(i); // 因为观测量和状态量维度相同，直接赋值
    }
    return measurement_sigma_points;
}

UnscentedKalmanFilter::UnscentedKalmanFilter()
{
    alpha = 0.001;
    beta = 2;
    kappa = 0;
    lambda = alpha * alpha * (STATE_DIM + kappa) - STATE_DIM;
    weights_m = Eigen::VectorXd::Zero(SIGMA_POINTS);
    weights_c = Eigen::VectorXd::Zero(SIGMA_POINTS);
    weights_m(0) = lambda / (STATE_DIM + lambda);
    weights_c(0) = lambda / (STATE_DIM + lambda) + (1 - alpha * alpha + beta);
    for (int i = 1; i < SIGMA_POINTS; ++i)
    {
        weights_m(i) = 0.5 / (STATE_DIM + lambda);
        weights_c(i) = 0.5 / (STATE_DIM + lambda);
    }
    sigma_points = Eigen::MatrixXd::Zero(STATE_DIM, SIGMA_POINTS);
}

// 初始化滤波器
void UnscentedKalmanFilter::initialize(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance)
{
    state = initial_state;
    covariance = initial_covariance;
    process_noise = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.01;
    measurement_noise = Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM) * 0.05;
}

// 预测步骤
void UnscentedKalmanFilter::predict(double dt)
{
    generateSigmaPoints();
    Eigen::MatrixXd predicted_sigma_points = predictSigmaPoints(sigma_points, dt);
    computeMeanAndCovariance(predicted_sigma_points, state, covariance);
    covariance += process_noise;
}

// 更新步骤
void UnscentedKalmanFilter::update(const Eigen::VectorXd &measurement)
{
    Eigen::MatrixXd measurement_sigma_points = measurementModel(sigma_points);
    Eigen::VectorXd measurement_mean;
    Eigen::MatrixXd measurement_cov;
    computeMeanAndCovariance(measurement_sigma_points, measurement_mean, measurement_cov);
    measurement_cov += measurement_noise;

    Eigen::MatrixXd cross_cov = Eigen::MatrixXd::Zero(STATE_DIM, MEAS_DIM);
    for (int i = 0; i < SIGMA_POINTS; ++i)
    {
        Eigen::VectorXd state_diff = sigma_points.col(i) - state;
        Eigen::VectorXd meas_diff = measurement_sigma_points.col(i) - measurement_mean;
        cross_cov += weights_c(i) * state_diff * meas_diff.transpose();
    }

    Eigen::MatrixXd kalman_gain = cross_cov * measurement_cov.inverse();
    state += kalman_gain * (measurement - measurement_mean);
    covariance -= kalman_gain * measurement_cov * kalman_gain.transpose();
}

// 预测未来位置
std::vector<Eigen::VectorXd> UnscentedKalmanFilter::predictFuturePositions(int steps, double dt)
{
    std::vector<Eigen::VectorXd> predictions;
    Eigen::VectorXd current_state = state;
    for (int i = 0; i < steps; ++i)
    {
        // 更新预测未来位置的公式以包含加速度
        current_state(0) += current_state(2) * dt + 0.5 * current_state(4) * dt * dt;
        current_state(1) += current_state(3) * dt + 0.5 * current_state(5) * dt * dt;
        predictions.emplace_back(current_state);
    }
    return predictions;
}
