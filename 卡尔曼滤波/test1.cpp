#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// 定义时间步长
const float dt = 0.1f;  // 时间步长，假设为0.1秒
const float processNoise = 1e-5f;  // 过程噪声
const float measurementNoise = 1e-1f;  // 测量噪声

class KalmanFilter {
public:
    // 初始化状态向量和协方差矩阵
    Vector4f state;       // 状态向量 [x, y, vx, vy]
    Matrix4f covMatrix;   // 状态协方差矩阵

    KalmanFilter() {
        // 初始状态 [x, y, vx, vy]
        state = Vector4f(0, 0, 0, 0);

        // 初始协方差矩阵，初始化为单位矩阵
        covMatrix = Matrix4f::Identity() * 1000;

        // 初始化状态转移矩阵 F
        F = Matrix4f::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        // 初始化过程噪声协方差矩阵 Q
        Q = Matrix4f::Identity() * processNoise;

        // 初始化测量矩阵 H （假设我们能测量位置 [x, y]）
        H = Matrix<float, 2, 4>::Zero();
        H(0, 0) = 1;  // 测量位置 x
        H(1, 1) = 1;  // 测量位置 y

        // 初始化测量噪声协方差矩阵 R
        R = Matrix2f::Identity() * measurementNoise;
    }

    // 预测阶段
    void predict() {
        // 预测状态：x' = F * x
        state = F * state;

        // 预测协方差矩阵：P' = F * P * F^T + Q
        covMatrix = F * covMatrix * F.transpose() + Q;
    }

    // 更新阶段（带有测量值 z 的更新）
    void update(const Vector2f& measurement) {
        // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
        Matrix2f S = H * covMatrix * H.transpose() + R;
        Matrix<float, 4, 2> K = covMatrix * H.transpose() * S.inverse();

        // 更新状态向量：x = x + K * (z - H * x)
        Vector2f y = measurement - H * state;  // 测量残差
        state = state + K * y;

        // 更新协方差矩阵：P = (I - K * H) * P
        Matrix4f I = Matrix4f::Identity();
        covMatrix = (I - K * H) * covMatrix;
    }

    // 打印当前状态
    void printState() {
        cout << "x: " << state(0) << ", y: " << state(1)
             << ", vx: " << state(2) << ", vy: " << state(3) << endl;
    }

private:
    Matrix4f F;  // 状态转移矩阵
    Matrix4f Q;  // 过程噪声协方差矩阵
    Matrix<float, 2, 4> H;  // 测量矩阵
    Matrix2f R;  // 测量噪声协方差矩阵
};

int main() {
    KalmanFilter kf;

    // 模拟一些测量值 [x, y]
    Vector2f measurement1(5.0, 10.0);
    Vector2f measurement2(6.0, 12.0);
    Vector2f measurement3(7.0, 14.0);

    // 进行卡尔曼滤波
    cout << "Initial state: " << endl;
    kf.printState();

    // 第一轮预测和更新
    kf.predict();
    kf.update(measurement1);
    cout << "After first measurement: " << endl;
    kf.printState();

    // 第二轮预测和更新
    kf.predict();
    kf.update(measurement2);
    cout << "After second measurement: " << endl;
    kf.printState();

    // 第三轮预测和更新
    kf.predict();
    kf.update(measurement3);
    cout << "After third measurement: " << endl;
    kf.printState();

    return 0;
}
