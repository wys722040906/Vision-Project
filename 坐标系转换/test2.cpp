#include <iostream>
#include <cmath>

/*

归一化四元数（如果需要）：
𝑞=𝑞/∣𝑞∣=𝑤+𝑥𝑖+𝑦𝑗+𝑧𝑘/sqrt(𝑤**2+𝑥**2+𝑦**2+𝑧**2)
    确保四元数是单位四元数。
​
旋转角度：
    θ=2⋅arccos(w)
​
旋转轴：
    if x = 0 and y = 0, then z = 1, then the rotation axis is none or all z-axis.
    else:
        axis=1/(x**2+y**2+z**2) * (x,y,z)
*/



struct Quaternion {
    double w, x, y, z;

    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // 计算旋转角度和旋转轴
    void getRotationAngleAndAxis(double &angle, double &axisX, double &axisY, double &axisZ) {
        // 计算旋转角度
        angle = 2 * acos(w);

        // 计算旋转轴
        double norm = sqrt(x * x + y * y + z * z);
        if (norm > 0) {
            axisX = x / norm;
            axisY = y / norm;
            axisZ = z / norm;
        } else {
            // 处理无旋转情况
            axisX = axisY = axisZ = 0; 
        }
    }
};

int main() {
    Quaternion q(0.707, 0.0, 0.707, 0.0); // 示例单位四元数
    double angle, axisX, axisY, axisZ;
    q.getRotationAngleAndAxis(angle, axisX, axisY, axisZ);
    
    std::cout << "Rotation Angle (radians): " << angle << std::endl;
    std::cout << "Rotation Axis: (" << axisX << ", " << axisY << ", " << axisZ << ")" << std::endl;

    return 0;
}
