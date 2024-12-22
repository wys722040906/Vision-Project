#include "track/kalmanfilter.h"


namespace detectAndTrack {

void ExtendedKalmanFilter::updateParam(const std::string& paramFile){
    ParameterReader paramLoad(paramFile);
    paramLoad.read("stateSize", stateSize);
    paramLoad.read("measurementSize", measurementSize);
    paramLoad.read("dt", timeStep);
    float Q_value, R_value;
    paramLoad.read("Q_value", Q_value);
    paramLoad.read("R_value", R_value);

    state = Eigen::VectorXf::Zero(stateSize);
    covMatrix = Eigen::MatrixXf::Identity(stateSize, stateSize) * 1000;
    Q = Eigen::MatrixXf::Identity(stateSize, stateSize)* Q_value;
    R = Eigen::MatrixXf::Identity(measurementSize, measurementSize)* R_value;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(int stateSize, int measurementSize, float timeStep,  float QValue, float RValue)
    : stateSize(stateSize),
      measurementSize(measurementSize),
      timeStep(timeStep),
      state(Eigen::VectorXf::Zero(stateSize)),
      covMatrix(Eigen::MatrixXf::Identity(stateSize, stateSize) * 1000),
      Q(Eigen::MatrixXf::Identity(stateSize, stateSize)),
      R(Eigen::MatrixXf::Identity(measurementSize, measurementSize)) {
        Q = Eigen::MatrixXf::Identity(stateSize, stateSize)* QValue;
        R = Eigen::MatrixXf::Identity(measurementSize, measurementSize)* RValue;
}

void ExtendedKalmanFilter::setTransitionFunction(const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& transitionFunction) {
    F = transitionFunction;
}

void ExtendedKalmanFilter::setMeasurementFunction(const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& measurementFunction) {
    H = measurementFunction;
}


void ExtendedKalmanFilter::predict() {
    state = F(state);
    Eigen::MatrixXf F_jacobian = computeJacobianF(state);
    covMatrix = F_jacobian * covMatrix * F_jacobian.transpose() + Q;
}


void ExtendedKalmanFilter::update(const Eigen::VectorXf& measurement) {
    Eigen::VectorXf z_hat = H(state);
    Eigen::VectorXf y = measurement - z_hat;
    Eigen::MatrixXf H_jacobian = computeJacobianH(state);
    Eigen::MatrixXf S = H_jacobian * covMatrix * H_jacobian.transpose() + R;
    Eigen::MatrixXf K = covMatrix * H_jacobian.transpose() * S.inverse();
    state = state + K * y;
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(covMatrix.rows(), covMatrix.cols());
    covMatrix = (I - K * H_jacobian) * covMatrix;
}


std::vector<double> ExtendedKalmanFilter::printState() const {
    // std::cout << "预测值: " << state.transpose() << std::endl;
    std::vector<double> result;
    if (state.size() >= 3) {
        double x = state(0);
        double y = state(1);
        double z = state(2);
        double distance = std::sqrt(x * x + y * y + z * z); // 计算距离
        result.push_back(x);
        result.push_back(y);
        result.push_back(z);
        result.push_back(distance);
    }
    return result;
}

// 计算雅可比矩阵 F
Eigen::MatrixXf ExtendedKalmanFilter::computeJacobianF(const Eigen::VectorXf& state) {
    Eigen::MatrixXf jacobian = Eigen::MatrixXf::Identity(state.size(), state.size());
    jacobian(0, 3) = timeStep;
    jacobian(1, 4) = timeStep;
    jacobian(2, 5) = timeStep;

    return jacobian;
}

// 计算雅可比矩阵 H 
Eigen::MatrixXf ExtendedKalmanFilter::computeJacobianH(const Eigen::VectorXf& state) {
    Eigen::MatrixXf jacobian(measurementSize, stateSize);
    jacobian.setZero();
    jacobian(0, 0) = 1;
    jacobian(1, 1) = 1;
    jacobian(2, 2) = 1;

    return jacobian;
}



} // namespace detectAndTrack
