#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <functional>
#include <iostream>
#include <vector>
#include <detect/paramLoad.h>
#include <string>

//创建示例：ExtendedKalmanFilter ekf(stateSize, measurementSize, dt);

namespace detectAndTrack {
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() = default;
    ~ExtendedKalmanFilter() = default;
    ExtendedKalmanFilter(int stateSize, int measurementSize, float timeStep = 0.05f, float QValue = 0.1f, float RValue = 0.1f);
    void updateParam(const std::string& paramFile);

    void setTransitionFunction(const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& transitionFunction);
    void setMeasurementFunction(const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& measurementFunction);

    void predict();
    void update(const Eigen::VectorXf& measurement);
    std::vector<double> printState() const;

public:
    float timeStep;                      // 时间步长

private:
    int stateSize;                       // 状态大小
    int measurementSize;                 // 测量大小
    Eigen::VectorXf state;               // 状态向量
    Eigen::MatrixXf covMatrix;           // 状态协方差矩阵
    Eigen::MatrixXf Q;                   // 过程噪声协方差矩阵
    Eigen::MatrixXf R;                   // 测量噪声协方差矩阵
    std::function<Eigen::VectorXf(const Eigen::VectorXf&)> F;  // 状态转移函数
    std::function<Eigen::VectorXf(const Eigen::VectorXf&)> H;  // 测量函数

    // 计算雅可比矩阵 F 
    Eigen::MatrixXf computeJacobianF(const Eigen::VectorXf& state);

    // 计算雅可比矩阵 H
    Eigen::MatrixXf computeJacobianH(const Eigen::VectorXf& state);
};

    // // 示例数据：连续帧的3D坐标 (x, y, z)
    // std::vector<std::vector<double>> points = {
    //     {0.0, 0.0, 0.0},
    //     {1.0, 2.0, 3.0},
    //     {2.0, 4.0, 6.0},
    //     {3.0, 6.0, 9.0},
    //     {4.0, 8.0, 12.0}
    // };

    // // 初始化样条插值对象
    // CubicSpline spline(points);

    // // 获取给定x值位置的插值结果
    // double xVal = 2.5;  // 需要预测的位置
    // std::vector<double> interpolatedPos = spline.interpolate(xVal);

    // std::cout << "Interpolated position at x = " << xVal << ": "
    //           << "y = " << interpolatedPos[0] << ", "
    //           << "z = " << interpolatedPos[1] << ", "
    //           << "z = " << interpolatedPos[2] << std::endl;


#endif // EXTENDED_KALMAN_FILTER_H


} // namespace detectAndTrack