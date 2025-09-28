import open3d as o3d
import numpy as np
import cv2
import yaml

# ----------------------------
# 1. 从 YAML 文件读取配置
# ----------------------------
with open("config.yaml", 'r') as f:
    cfg = yaml.safe_load(f)

pcd_path      = cfg["pcd_path"]
img_path      = cfg["img_path"]
camera_matrix = np.array(cfg["camera_matrix"], dtype=np.float64)
dist_coeffs   = np.array(cfg["dist_coeffs"],   dtype=np.float64)
R             = np.array(cfg["R"],             dtype=np.float64)
t             = np.array(cfg["t"],             dtype=np.float64).reshape(3,1)

# ----------------------------
# 2. 读取点云和图像
# ----------------------------
pcd = o3d.io.read_point_cloud(pcd_path)
pts_lidar = np.asarray(pcd.points)  # (N,3)

img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
if img is None:
    raise FileNotFoundError(f"Cannot read image: {img_path}")
h, w = img.shape[:2]

# ----------------------------
# 3. 坐标变换：LiDAR -> Camera
# ----------------------------
pts_cam = (R @ pts_lidar.T + t).T   # (N,3)
mask    = pts_cam[:,2] > 0          # 只保留相机前方的点
pts_cam = pts_cam[mask]

# ----------------------------
# 4. 投影到像素平面
# ----------------------------
rvec = np.zeros((3,1), dtype=np.float64)
tvec = np.zeros((3,1), dtype=np.float64)

proj, _ = cv2.projectPoints(
    objectPoints=pts_cam,
    rvec=rvec, tvec=tvec,
    cameraMatrix=camera_matrix,
    distCoeffs=dist_coeffs
)
uv = proj.reshape(-1,2)  # (M,2)
depth = pts_cam[:,2]     # (M,)

# ----------------------------
# 5. 构建深度图
# ----------------------------
# 用 np.inf 初始化，然后取最小深度
depth_img = np.full((h, w), np.inf, dtype=np.float32)

for (u, v), z in zip(uv, depth):
    ui, vi = int(round(u)), int(round(v))
    if 0 <= ui < w and 0 <= vi < h:
        if z < depth_img[vi, ui]:
            depth_img[vi, ui] = z

# 把未被赋值的位置设为 0
depth_img[depth_img == np.inf] = 0

# （可选）为了可视化，归一化到 0–255
vis = depth_img.copy()
max_depth = np.max(vis)
if max_depth > 0:
    vis = (vis / max_depth * 255).astype(np.uint8)
else:
    vis = np.zeros_like(vis, dtype=np.uint8)

# ----------------------------
# 6. 显示 & 保存
# ----------------------------
cv2.imshow("Raw Depth (float meters)", depth_img)    # 以米为单位的原始深度
cv2.imshow("Depth Visualization", vis)               # 归一化后可视化
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存
cv2.imwrite("depth_raw.exr", depth_img)    # EXR 可保存浮点图
cv2.imwrite("depth_vis.png", vis)          # PNG 可视化图
