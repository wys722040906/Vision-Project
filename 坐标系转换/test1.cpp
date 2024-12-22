#include <iostream>
#include <cmath>

/*
定义欧拉角：绕 Z 轴旋转（Yaw），绕 Y 轴旋转（Pitch），绕 X 轴旋转（Roll）

四元数：由 w + xi + yj + zk 组成，i^2 = j^2 = k^2 = ijk = -1，w^2 + x^2 + y^2 + z^2 = 1

四元数转欧拉角：

    # Yaw (Z轴旋转)
    yaw = np.arctan2(2 * (y * w + x * z), 1 - 2 * (y**2 + z**2))
    
    # Pitch (Y轴旋转)
    pitch = np.arcsin(2 * (x * y - z * w))
    
    # Roll (X轴旋转)
    roll = np.arctan2(2 * (x * w + y * z), 1 - 2 * (x**2 + y**2))
*/


struct Quaternion {
    double w, x, y, z;

    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    void toEulerAngles(double &roll, double &pitch, double &yaw) {
        // 计算欧拉角
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp); // Roll (X轴)

        double sinp = 2 * (w * y - z * x);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // 使用90度限制
        else
            pitch = asin(sinp); // Pitch (Y轴)

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp); // Yaw (Z轴)
    }
};

int main() {
    Quaternion q(0.707, 0.0, 0.707, 0.0); // 示例单位四元数
    double roll, pitch, yaw;
    q.toEulerAngles(roll, pitch, yaw);
    
    std::cout << "Roll (radians): " << roll << std::endl;
    std::cout << "Pitch (radians): " << pitch << std::endl;
    std::cout << "Yaw (radians): " << yaw << std::endl;

    return 0;
}
