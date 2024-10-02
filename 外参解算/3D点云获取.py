import cv2
import numpy as np
"""
激光扫描仪、结构光、立体视觉和多视图几何    
    使用立体视觉和OpenCV进行三维重建
"""
def stereo_reconstruction(left_image_path, right_image_path, camera_matrix_left, camera_matrix_right, dist_coeffs_left, dist_coeffs_right, R, T):
    # 读取左、右图像
    left_img = cv2.imread(left_image_path, cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imread(right_image_path, cv2.IMREAD_GRAYSCALE)

    # 计算视差图
    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(left_img, right_img)

    # 将视差图转换为深度图
    disparity = np.float32(disparity) / 16.0

    # 将视差图转换为三维点云
    Q = np.float32([
        [1, 0, 0, -0.5 * left_img.shape[1]],
        [0, -1, 0, 0.5 * left_img.shape[0]],
        [0, 0, 0, -camera_matrix_left[0, 0]],
        [0, 0, 1, 0]
    ])

    points_3D = cv2.reprojectImageTo3D(disparity, Q)
    mask_map = disparity > disparity.min()

    output_points = points_3D[mask_map]

    return output_points

# 示例相机参数（这些需要通过相机校准获取）
camera_matrix_left = np.array([[fx1, 0, cx1],
                               [0, fy1, cy1],
                               [0, 0, 1]], dtype=np.float32)

camera_matrix_right = np.array([[fx2, 0, cx2],
                                [0, fy2, cy2],
                                [0, 0, 1]], dtype=np.float32)

dist_coeffs_left = np.zeros((5, 1))  # 假设没有畸变
dist_coeffs_right = np.zeros((5, 1))

# 两台相机的旋转和平移关系（通过立体校正获取）
R = np.eye(3, dtype=np.float32)  # 旋转矩阵
T = np.array([[baseline], [0], [0]], dtype=np.float32)  # 平移向量

# 示例左右图像路径
left_image_path = 'left_image.png'
right_image_path = 'right_image.png'

# 获取三维点云
points_3D = stereo_reconstruction(left_image_path, right_image_path, camera_matrix_left, camera_matrix_right, dist_coeffs_left, dist_coeffs_right, R, T)

print("3D Points:", points_3D)
