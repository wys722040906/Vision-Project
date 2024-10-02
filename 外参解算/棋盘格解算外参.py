import cv2
import numpy as np

# 定义棋盘格尺寸和每个方块的实际尺寸（例如 20 毫米）
chessboard_size = (7, 9)
square_size = 25  # 每个方块的边长

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 准备对象点，例如 (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# 将单位从格子转换为实际尺寸
objp *= square_size

objpoints = []  # 3D 点在真实世界中的坐标
imgpoints = []  # 2D 点在图像平面中的坐标

# 假设已经有相机内参矩阵 mtx 和畸变系数 dist
mtx = np.array([[441.147220, 0, 321.594855],  # 内参矩阵
                [0, 406.092110, 241.403195],
                [0, 0, 1]])
dist = np.array([0.1843138, -0.2167312, 0.004313, -0.0462188, 0])

images = ['/home/wys/Documents/cam/7/left-0000.png']  # 包含所有棋盘格图片的列表

for image_path in images:
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # 可视化角点检测结果
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# 相机校准
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 打印结果
print("内参矩阵 (mtx):", mtx)
print("畸变系数 (dist):", dist)

# 计算每张图片的外参
for i in range(len(images)):
    # 对于每张图片，使用相应的2D图像点和3D对象点计算外参
    success, rvec, tvec = cv2.solvePnP(objpoints[i], imgpoints[i], mtx, dist)
    
    if success:
        R, _ = cv2.Rodrigues(rvec)  # 将旋转向量转换为旋转矩阵
        t = tvec
        print(f"图像 {i} 的旋转矩阵 (R):\n", R)
        print(f"图像 {i} 的平移向量 (t):\n", t)

# rvecs 和 tvecs 包含了每张图片的旋转向量和平移向量
# 假设我们对第一个视角进行处理
rvec = rvecs[0]
tvec = tvecs[0]

# 将旋转向量转换为旋转矩阵
R, _ = cv2.Rodrigues(rvec)
t = tvec

print("旋转矩阵 (R):", R)
print("平移向量 (t):", t)
