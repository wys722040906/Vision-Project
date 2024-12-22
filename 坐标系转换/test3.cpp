#include <iostream>
#include <cmath>

struct Quaternion {
    double w, x, y, z;
    
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
};

Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
    // 计算半角
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    // 四元数分量
    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;

    return Quaternion(w, x, y, z);
}

int main() {
    double roll = 0;        // 绕 X 轴的旋转角度（Roll）
    double pitch = 1.54622; // 绕 Y 轴的旋转角度（Pitch）
    double yaw = 0;         // 绕 Z 轴的旋转角度（Yaw）

    Quaternion q = eulerToQuaternion(roll, pitch, yaw);
    
    std::cout << "Quaternion:" << std::endl;
    std::cout << "w: " << q.w << std::endl;
    std::cout << "x: " << q.x << std::endl;
    std::cout << "y: " << q.y << std::endl;
    std::cout << "z: " << q.z << std::endl;

    return 0;
}
