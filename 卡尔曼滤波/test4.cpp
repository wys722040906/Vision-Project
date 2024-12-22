#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;
using namespace std;

class ExtendedKalmanFilter {
public:
    Vector4f state;        // 状态向量 [x, y, vx, vy]
    Matrix4f covMatrix;    // 协方差矩阵
    Matrix4f F;            // 状态转移矩阵
    Matrix4f Q;            // 过程噪声协方差矩阵
    Matrix2f R;            // 测量噪声协方差矩阵

    ExtendedKalmanFilter() {
        // 初始状态
        state = Vector4f(1, 1, 1, 1);

        // 状态协方差矩阵初始化
        covMatrix = Matrix4f::Identity() * 1000;

        // 状态转移矩阵
        F = Matrix4f::Identity();
        F(0, 2) = 1.0f;  // 时间步长 dt 假设为1
        F(1, 3) = 1.0f;

        // 过程噪声
        Q = Matrix4f::Identity() * 1e-3;

        // 测量噪声
        R = Matrix2f::Identity() * 1e-1;
    }

    // 预测步骤
    void predict() {
        state = F * state;
        covMatrix = F * covMatrix * F.transpose() + Q;
    }

    // 更新步骤
    void update(const Vector2f& measurement) {
        // 计算非线性观测模型的雅可比矩阵 H_j
        MatrixXf H_j(2, 4);
        float x = state(0), y = state(1);
        float range = sqrt(x * x + y * y);
        
        if (range < 1e-4) return;  // 避免除以零
        
        H_j(0, 0) = x / range;
        H_j(0, 1) = y / range;
        H_j(0, 2) = 0;
        H_j(0, 3) = 0;
        
        H_j(1, 0) = -y / (range * range);
        H_j(1, 1) = x / (range * range);
        H_j(1, 2) = 0;
        H_j(1, 3) = 0;

        // 预测观测值
        Vector2f predicted_measurement;
        predicted_measurement(0) = range;
        predicted_measurement(1) = atan2(y, x);

        // 计算卡尔曼增益
        Matrix2f S = H_j * covMatrix * H_j.transpose() + R;
        MatrixXf K = covMatrix * H_j.transpose() * S.inverse();

        // 更新状态
        Vector2f y_residual = measurement - predicted_measurement;
        state = state + K * y_residual;

        // 更新协方差
        Matrix4f I = Matrix4f::Identity();
        covMatrix = (I - K * H_j) * covMatrix;
    }

    // 打印状态
    void printState() {
        cout << "x: " << state(0) << ", y: " << state(1)
             << ", vx: " << state(2) << ", vy: " << state(3) << endl;
    }
};

int main() {
    ExtendedKalmanFilter ekf;

    // 模拟一些测量值 [r, theta]
    Vector2f measurement1(5.0, 0.9273);  // r = 5, theta = pi/4
    Vector2f measurement2(6.0, 0.9827);  // r = 6, theta = pi/3

    // 初始状态
    cout << "Initial state: " << endl;
    ekf.printState();

    // 第一次预测和更新
    ekf.predict();
    ekf.update(measurement1);
    cout << "After first measurement: " << endl;
    ekf.printState();

    // 第二次预测和更新
    ekf.predict();
    ekf.update(measurement2);
    cout << "After second measurement: " << endl;
    ekf.printState();

    return 0;
}
