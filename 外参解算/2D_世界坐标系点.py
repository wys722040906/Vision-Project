"""
以下是一个完整的示例代码，展示如何从像素坐标和距离计算物体在世界坐标系中的 x、y、z 偏移量：
"""

import numpy as np

def calculate_world_coordinates(u, v, Z, K, R, t):
    """
    计算物体在世界坐标系中的x、y、z偏移量。

    参数：
    u, v: 物体在图像中的像素坐标。
    Z: 物体到相机的距离。
    K: 相机内参矩阵。
    R: 相机的旋转矩阵。
    t: 相机的平移向量。

    返回值：
    X_w, Y_w, Z_w: 物体在世界坐标系中的坐标。
    """
    # 从像素坐标转换到相机坐标
    f_x = K[0, 0]
    f_y = K[1, 1]
    c_x = K[0, 2]
    c_y = K[1, 2]

    X_c = (u - c_x) * Z / f_x
    Y_c = (v - c_y) * Z / f_y
    Z_c = Z
    
    P_c = np.array([X_c, Y_c, Z_c, 1])

    # 从相机坐标转换到世界坐标
    RT = np.hstack((R, t.reshape(-1, 1)))
    RT = np.vstack((RT, [0, 0, 0, 1]))

    P_w = np.dot(RT, P_c)

    return P_w[0], P_w[1], P_w[2]

# 示例内参矩阵（根据你的相机参数进行设置）
K = np.array([[800, 0, 320],
              [0, 800, 240],
              [0, 0, 1]])

# 示例旋转矩阵和平移向量（根据你的相机姿态设置）
R = np.array([[0.866, -0.5, 0],
              [0.5, 0.866, 0],
              [0, 0, 1]])
t = np.array([1.0, 2.0, 3.0])

# 示例像素坐标和物体到相机的距离
u = 320  # 物体在图像中的u坐标
v = 240  # 物体在图像中的v坐标
Z = 5.0  # 物体到相机的距离（米）

# 计算物体在世界坐标系中的坐标
X_w, Y_w, Z_w = calculate_world_coordinates(u, v, Z, K, R, t)
print(f"世界坐标系中的坐标: X = {X_w}, Y = {Y_w}, Z = {Z_w}")
