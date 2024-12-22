#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class KalmanFilter {
public:
    // 构造函数，初始化卡尔曼滤波器
    KalmanFilter(int stateSize, int measurementSize)
        : state(VectorXf::Zero(stateSize)),
          covMatrix(MatrixXf::Identity(stateSize, stateSize) * 1000) {

        // 初始化状态转移矩阵 F
        F = MatrixXf::Identity(stateSize, stateSize);
        
        // 初始化过程噪声协方差矩阵 Q
        Q = MatrixXf::Identity(stateSize, stateSize);

        // 初始化测量矩阵 H
        H = MatrixXf::Zero(measurementSize, stateSize);

        // 初始化测量噪声协方差矩阵 R
        R = MatrixXf::Identity(measurementSize, measurementSize);
    }

    // 设置状态转移矩阵 F
    void setTransitionMatrix(const MatrixXf& transitionMatrix) {
        F = transitionMatrix;
    }

    // 设置过程噪声协方差矩阵 Q
    void setProcessNoiseCovariance(const MatrixXf& processNoiseCovariance) {
        Q = processNoiseCovariance;
    }

    // 设置测量矩阵 H
    void setMeasurementMatrix(const MatrixXf& measurementMatrix) {
        H = measurementMatrix;
    }

    // 设置测量噪声协方差矩阵 R
    void setMeasurementNoiseCovariance(const MatrixXf& measurementNoiseCovariance) {
        R = measurementNoiseCovariance;
    }

    // 预测阶段
    void predict() {
        // 预测状态：x' = F * x
        state = F * state;

        // 预测协方差矩阵：P' = F * P * F^T + Q
        covMatrix = F * covMatrix * F.transpose() + Q;
    }

    // 更新阶段（带有测量值 z 的更新）
    void update(const VectorXf& measurement) {
        // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
        MatrixXf S = H * covMatrix * H.transpose() + R;
        MatrixXf K = covMatrix * H.transpose() * S.inverse();

        // 更新状态向量：x = x + K * (z - H * x)
        VectorXf y = measurement - H * state;  // 测量残差
        state = state + K * y;

        // 更新协方差矩阵：P = (I - K * H) * P
        MatrixXf I = MatrixXf::Identity(covMatrix.rows(), covMatrix.cols());
        covMatrix = (I - K * H) * covMatrix;
    }

    // 打印当前状态
    void printState() const {
        cout << "State: " << state.transpose() << endl;
    }

private:
    VectorXf state;       // 状态向量
    MatrixXf covMatrix;   // 状态协方差矩阵
    MatrixXf F;           // 状态转移矩阵
    MatrixXf Q;           // 过程噪声协方差矩阵
    MatrixXf H;           // 测量矩阵
    MatrixXf R;           // 测量噪声协方差矩阵
};

// 外部初始化示例
int main() {
    // 定义状态和测量的维度
    int stateSize = 6; // [x, y, z, vx, vy, vz]
    int measurementSize = 3; // [x, y, z]

    // 创建卡尔曼滤波器实例
    KalmanFilter kf(stateSize, measurementSize);

    // 设置状态转移矩阵 F
    float dt = 0.1f;  // 时间步长
    MatrixXf F(stateSize, stateSize);
    F << 1, 0, 0, dt, 0, 0,  // x 更新
        0, 1, 0, 0, dt, 0,  // y 更新
        0, 0, 1, 0, 0, dt,  // z 更新
        0, 0, 0, 1, 0, 0,   // vx 保持
        0, 0, 0, 0, 1, 0,   // vy 保持
        0, 0, 0, 0, 0, 1;   // vz 保持

    kf.setTransitionMatrix(F);

    // 设置过程噪声协方差矩阵 Q -- 这里假设过程噪声是高斯白噪声
    // 实际 -- 微调
    /*
        // 设置过程噪声协方差矩阵 Q
        MatrixXf Q = MatrixXf::Zero(stateSize, stateSize);
        // 设置位置的过程噪声
        Q(0, 0) = 0.1;  // x 方向位置噪声
        Q(1, 1) = 0.1;  // y 方向位置噪声
        Q(2, 2) = 0.1;  // z 方向位置噪声
        // 设置速度的过程噪声
        Q(3, 3) = 1.0;  // x 方向速度噪声
        Q(4, 4) = 1.0;  // y 方向速度噪声
        Q(5, 5) = 1.0;  // z 方向速度噪声
    */
    MatrixXf Q = MatrixXf::Identity(stateSize, stateSize) * 0.1;
    kf.setProcessNoiseCovariance(Q);

    // 设置测量矩阵 H 行数--测量值个数，列数-状态个数
    // 这里假设测量值是三维坐标 [x, y, z]
    MatrixXf H(measurementSize, stateSize);
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;
    kf.setMeasurementMatrix(H);

    // 设置测量噪声协方差矩阵 R
    float measurementNoiseValue = 0.1f;  // 根据传感器规格或经验值
    //  波动 + R  平滑 -R   
    MatrixXf R = MatrixXf::Identity(measurementSize, measurementSize) * (measurementNoiseValue * measurementNoiseValue);
    kf.setMeasurementNoiseCovariance(R);




    // 模拟一些三维测量值 [x, y, z]
    VectorXf measurement1(3);
    measurement1 << 5.0, 10.0, 3.0;
    VectorXf measurement2(3);
    measurement2 << 6.0, 12.0, 3.5;
    VectorXf measurement3(3);
    measurement3 << 7.0, 14.0, 4.0;

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
