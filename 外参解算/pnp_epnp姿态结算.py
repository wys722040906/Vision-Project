import cv2
import numpy as np

def get_camera_pose(object_points, image_points, camera_matrix):
    """
    计算相机姿态（旋转矩阵和平移向量）
    """
    dist_coeffs = np.zeros((4, 1))  # 假设没有畸变
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    return rotation_matrix, tvec


def pixel_to_world(u, v, Zc, camera_matrix, rotation_matrix, translation_vector):
    """
    将像素坐标转换到世界坐标系
    """
    # 逆相机内参矩阵
    inv_camera_matrix = np.linalg.inv(camera_matrix)

    # 从像素坐标转换到相机坐标
    uv1 = np.array([u, v, 1.0])
    cam_coords = Zc * inv_camera_matrix.dot(uv1)

    # 从相机坐标转换到世界坐标
    inv_rotation_matrix = np.linalg.inv(rotation_matrix)
    world_coords = inv_rotation_matrix.dot(cam_coords - translation_vector.flatten())

    return world_coords


# 示例相机内参矩阵
camera_matrix = np.array([[476.2517996, 0, 312.809844],
                          [0, 437.5255194, 251.9442406],
                          [0, 0, 1]], dtype=np.float32)

# 示例三维点和对应的二维点
#0.0915
#
import numpy as np
object_points = np.array([[3.911,5.572,-0.0085],
                          [0.190,5.782,-0.0085],
                          [1.960,5.457,-0.0085],
                          [1.376,5.310,-0.0085],
                          [0.80,4.17,-0.0085],
                          [1.057,3.815,-0.0085],
                          [1.822,4.288,-0.0085],
                          [2.094,4.765,-0.0085],
                          [2.051,3.920,-0.0085],
                          [3.071,4.652,-0.0085],
                          [2.296,5.580,-0.0085],
                          [3.372,3.696,-0.0085],
                          [1.643,1.490,0.0915],
                          [1.920,2.030,0.0915],
                          [2.315,2.445,0.0915],
                          [2.114,1.850,0.0915],
                          [2.02,1.19,0.0915],
                          [1.175,2.44,0.0915],
                          [2.63,2.18,0.0915],
                          [2.655,1.840,0.0915]
], dtype=np.float32)


image_points = np.array([[127,43],
                         [488,53],
                         [322,54],
                         [382,58],
                         [442,65],
                         [445,95],
                         [340,76],
                         [297,63],
                         [241,83],
                         [183,60],
                         [220,44],
                         [101,84],
                         [473,280],
                         [338,182],
                         [234,142],
                         [241,204],
                         [309,380],
                         [145,440],
                         [142,158],
                         [88,204]], dtype=np.float32)

# 获取相机姿态
rotation_matrix, translation_vector = get_camera_pose(object_points, image_points, camera_matrix)

print(f"rotation_matrix:{rotation_matrix}")
print(f"translation_vector:{translation_vector}")
# # 示例像素点和深度值
# u, v = 320, 240  # 像素坐标
# Zc = 5.0         # 深度值

# # 转换到世界坐标系
# world_coords = pixel_to_world(u, v, Zc, camera_matrix, rotation_matrix, translation_vector)

# print("World Coordinates:", world_coords)