import torch
import torchvision.transforms as T
from PIL import Image
import cv2
import time

# 加载预训练的深度估计模型（示例使用MiDaS）
model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.eval()
model.to(device)

# 图像变换
transform = T.Compose([
    T.Resize(384),
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# 加载并预处理图像
img_path = "/home/wys/Documents/1.jpg"
img = Image.open(img_path)
img_tensor = transform(img).unsqueeze(0).to(device)  # 在转换为张量并添加维度后将其移动到设备上

# 预测深度
with torch.no_grad():
    start_time = time.time()
    depth = model(img_tensor)
    end_time = time.time()

inference_time = end_time - start_time
print(f"Inference time: {inference_time:.4f} seconds")

# 后处理并可视化深度图
depth = depth.squeeze().cpu().numpy()  # 将数据移回CPU并转换为numpy数组
depth = cv2.normalize(depth, None, 0, 1, cv2.NORM_MINMAX)
cv2.imshow("Depth Map", depth)
cv2.waitKey(0)
