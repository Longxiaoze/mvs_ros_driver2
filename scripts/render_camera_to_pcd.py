import open3d as o3d
import numpy as np
import cv2
import yaml

# ----------------------------
# 1. 从 config.yaml 读取配置
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
# 2. 读取点云 & 图像
# ----------------------------
pcd = o3d.io.read_point_cloud(pcd_path)
pts_lidar = np.asarray(pcd.points)  # (N,3)

img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
if img is None:
    raise FileNotFoundError(f"Cannot read image: {img_path}")
h, w = img.shape[:2]

# ----------------------------
# 3. 坐标变换：LiDAR → Camera
#    X_cam = R * X_lidar + t
# ----------------------------
pts_cam = (R @ pts_lidar.T + t).T    # (N,3)
mask    = pts_cam[:,2] > 0           # 只保留相机前方的点
pts_cam = pts_cam[mask]
pts_lidar = pts_lidar[mask]          # 同步过滤

# ----------------------------
# 4. 投影 & 采样颜色
# ----------------------------
rvec = np.zeros((3,1), dtype=np.float64)
tvec = np.zeros((3,1), dtype=np.float64)

proj, _ = cv2.projectPoints(
    objectPoints=pts_cam,
    rvec=rvec, tvec=tvec,
    cameraMatrix=camera_matrix,
    distCoeffs=dist_coeffs
)
uv = proj.reshape(-1,2)

colors = []
for u, v in uv:
    ui, vi = int(round(u)), int(round(v))
    if 0 <= ui < w and 0 <= vi < h:
        bgr = img[vi, ui]
        colors.append([bgr[2]/255., bgr[1]/255., bgr[0]/255.])
    else:
        colors.append([0.5, 0.5, 0.5])
colors = np.array(colors, dtype=np.float64)

# ----------------------------
# 5. 构建带色点云 & 可视化
# ----------------------------
pcd_colored = o3d.geometry.PointCloud()
pcd_colored.points = o3d.utility.Vector3dVector(pts_lidar)
pcd_colored.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries(
    [pcd_colored],
    window_name="Colored LiDAR→Camera PointCloud",
    width=1280, height=720
)

# 如需保存：
# o3d.io.write_point_cloud("colored_output.ply", pcd_colored)
