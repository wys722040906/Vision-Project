import numpy as np

def rotation_matrix_from_euler_angles(alpha, beta, gamma):
    # 计算绕X轴的旋转矩阵
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])
    d
    # 计算绕Y轴的旋转矩阵
    R_y = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])
    
    # 计算绕Z轴的旋转矩阵
    R_z = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])
    
    # 总旋转矩阵
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def transformation_matrix_from_parameters(tx, ty, tz, alpha, beta, gamma):
    # 计算旋转矩阵
    R = rotation_matrix_from_euler_angles(alpha, beta, gamma)
    
    # 平移向量
    t = np.array([tx, ty, tz])
    
    # 齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return T

# 示例参数
tx, ty, tz = -0.2,0, 0.01  # 平移量
alpha, beta, gamma = np.deg2rad(-90), np.deg2rad(0), np.deg2rad(0)  # 旋转角度（弧度制）

# 计算齐次变换矩阵
T = transformation_matrix_from_parameters(tx, ty, tz, alpha, beta, gamma)
print("旋转矩阵 R:\n", T[:3, :3])
print("平移向量 t:\n", T[:3, 3])
print("齐次变换矩阵 T:\n", T)
