#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class ExtendedKalmanFilter {
public:
    // 构造函数，初始化扩展卡尔曼滤波器
    ExtendedKalmanFilter(int stateSize, int measurementSize)
        : state(VectorXf::Zero(stateSize)),
          covMatrix(MatrixXf::Identity(stateSize, stateSize) * 1000) {

        // 初始化过程噪声协方差矩阵 Q
        Q = MatrixXf::Identity(stateSize, stateSize);

        // 初始化测量噪声协方差矩阵 R
        R = MatrixXf::Identity(measurementSize, measurementSize);
    }

    // 设置状态转移函数
    void setTransitionFunction(const function<VectorXf(const VectorXf&)>& transitionFunction) {
        F = transitionFunction;
    }

    // 设置过程噪声协方差矩阵 Q
    void setProcessNoiseCovariance(const MatrixXf& processNoiseCovariance) {
        Q = processNoiseCovariance;
    }

    // 设置测量函数
    void setMeasurementFunction(const function<VectorXf(const VectorXf&)>& measurementFunction) {
        H = measurementFunction;
    }

    // 设置测量噪声协方差矩阵 R
    void setMeasurementNoiseCovariance(const MatrixXf& measurementNoiseCovariance) {
        R = measurementNoiseCovariance;
    }

    // 预测阶段
    void predict() {
        // 预测状态
        state = F(state);

        // 计算雅可比矩阵
        MatrixXf F_jacobian = computeJacobianF(state);

        // 预测协方差矩阵
        covMatrix = F_jacobian * covMatrix * F_jacobian.transpose() + Q;
    }

    // 更新阶段（带有测量值 z 的更新）
    void update(const VectorXf& measurement) {
        // 计算测量预测
        VectorXf z_hat = H(state);

        // 计算测量残差
        VectorXf y = measurement - z_hat;

        // 计算雅可比矩阵
        MatrixXf H_jacobian = computeJacobianH(state);

        // 计算卡尔曼增益
        MatrixXf S = H_jacobian * covMatrix * H_jacobian.transpose() + R;
        MatrixXf K = covMatrix * H_jacobian.transpose() * S.inverse();

        // 更新状态
        state = state + K * y;

        // 更新协方差矩阵
        MatrixXf I = MatrixXf::Identity(covMatrix.rows(), covMatrix.cols());
        covMatrix = (I - K * H_jacobian) * covMatrix;
    }

    // 打印当前状态
    void printState() const {
        cout << "State: " << state.transpose() << endl;
    }

private:
    VectorXf state;       // 状态向量
    MatrixXf covMatrix;   // 状态协方差矩阵
    MatrixXf Q;           // 过程噪声协方差矩阵
    MatrixXf R;           // 测量噪声协方差矩阵
    function<VectorXf(const VectorXf&)> F;  // 状态转移函数
    function<VectorXf(const VectorXf&)> H;  // 测量函数

    // 计算雅可比矩阵 F 的函数
    MatrixXf computeJacobianF(const VectorXf& state) {
        // 这里需要根据状态转移函数定义雅可比矩阵
        // MatrixXf jacobian(state.size(), state.size());
        // jacobian.setIdentity(); // 应根据实际非线性关系定义
        MatrixXf jacobian = MatrixXf::Identity(state.size(), state.size());
        float dt = 0.05f;  // 假设 dt 已知
        jacobian(0, 3) = dt;
        jacobian(1, 4) = dt;
        jacobian(2, 5) = dt;
            
        // 示例：假设状态转移为线性情况
        // jacobian.setIdentity(); // 这里应该根据实际非线性关系设置
        return jacobian;
    }

    // 计算雅可比矩阵 H 的函数
    MatrixXf computeJacobianH(const VectorXf& state) {
        // 这里需要根据测量函数定义雅可比矩阵
        MatrixXf jacobian(H(state).size(), state.size());
        // 示例：假设测量函数为线性情况
        jacobian.setZero(); // 这里应该根据实际非线性关系设置
        // 例如，假设测量是某个状态的线性组合
        jacobian(0, 0) = 1; // 这里假设测量与状态相关
        jacobian(1, 1) = 1;
        jacobian(2, 2) = 1;
        return jacobian;
    }
};

// 外部初始化示例
int main() {
    // 定义状态和测量的维度
    int stateSize = 6; // [x, y, z, vx, vy, vz]
    int measurementSize = 3; // [x, y, z]

    // 创建扩展卡尔曼滤波器实例
    ExtendedKalmanFilter ekf(stateSize, measurementSize);

    // 设置状态转移函数
    float dt = 0.05f;  // 时间步长   // 高精度 数据稳定 低效率
    ekf.setTransitionFunction([dt](const VectorXf& state) {
        VectorXf newState(state.size());
        newState << 
            state(0) + state(3) * dt,  // x 更新
            state(1) + state(4) * dt,  // y 更新
            state(2) + state(5) * dt,  // z 更新
            state(3),                  // vx 保持
            state(4),                  // vy 保持
            state(5);                  // vz 保持
        return newState;
    });

    // 设置过程噪声协方差矩阵 Q(对角矩阵) -- 表示不确定性 -- 动态变化大 +Q(大噪声)   --保证单元尺度 量纲一致
    MatrixXf Q = MatrixXf::Identity(stateSize, stateSize) * 0.1;
    ekf.setProcessNoiseCovariance(Q);

    // 设置测量函数
    ekf.setMeasurementFunction([](const VectorXf& state) {
        return state.head<3>(); // 仅返回 [x, y, z]
    });

    // 设置测量噪声协方差矩阵 R
    float measurementNoiseValue = 0.1f;  // 根据传感器规格或经验值
    MatrixXf R = MatrixXf::Identity(measurementSize, measurementSize) * (measurementNoiseValue * measurementNoiseValue);
    ekf.setMeasurementNoiseCovariance(R);

    // 模拟一些三维测量值 [x, y, z]
    VectorXf measurement1(3);
    measurement1 << 5.0, 10.0, 3.0;
    VectorXf measurement2(3);
    measurement2 << 6.0, 12.0, 3.5;
    VectorXf measurement3(3);
    measurement3 << 7.0, 14.0, 4.0;

    // 进行扩展卡尔曼滤波
    cout << "Initial state: " << endl;
    ekf.printState();

    // 第一轮预测和更新
    ekf.predict();
    ekf.update(measurement1);
    cout << "After first measurement: " << endl;
    ekf.printState();

    // 第二轮预测和更新
    ekf.predict();
    ekf.update(measurement2);
    cout << "After second measurement: " << endl;
    ekf.printState();

    // 第三轮预测和更新
    ekf.predict();
    ekf.update(measurement3);
    cout << "After third measurement: " << endl;
    ekf.printState();

    return 0;
}
