import cv2
import numpy as np

# 加载图像
img1 = cv2.imread('image1.jpg', cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread('image2.jpg', cv2.IMREAD_GRAYSCALE)

# 创建ORB检测器和描述子
orb = cv2.ORB_create()

# 检测特征点和计算描述子
keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

# 创建匹配器
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# 匹配特征点
matches = bf.match(descriptors1, descriptors2)

# 提取匹配的特征点
src_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

# 计算基本矩阵并从中提取每个匹配点对的单应矩阵
F, mask = cv2.findFundamentalMat(src_pts, dst_pts, cv2.FM_LMEDS)

# 选择在前面检测到的特征点
src_pts = src_pts[mask.ravel() == 1]
dst_pts = dst_pts[mask.ravel() == 1]

# 使用三角法计算深度
def depth_estimation(kps1, kps2, K):
    pts1 = cv2.undistortPoints(kps1, K, D)
    pts2 = cv2.undistortPoints(kps2, K, D)
    pts1 = np.squeeze(pts1)
    K = np.float32(K)

 solve find_estimate Depth accurately
