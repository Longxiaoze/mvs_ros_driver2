import open3d as o3d
import numpy as np
import cv2
import yaml

# ----------------------------
# 1. 从 YAML 文件读取配置
# ----------------------------
with open("config.yaml", 'r') as f:
    cfg = yaml.safe_load(f)

pcd_path       = cfg["pcd_path"]
img_path       = cfg["img_path"]
camera_matrix  = np.array(cfg["camera_matrix"], dtype=np.float64)
dist_coeffs    = np.array(cfg["dist_coeffs"],   dtype=np.float64)
R              = np.array(cfg["R"],             dtype=np.float64)
t              = np.array(cfg["t"],             dtype=np.float64).reshape(3,1)

# ----------------------------
# 2. 读取点云和图像
# ----------------------------
pcd = o3d.io.read_point_cloud(pcd_path)
pts_lidar = np.asarray(pcd.points)  # (N,3)

img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"Cannot read image: {img_path}")

# ----------------------------
# 3. 点坐标变换：LiDAR -> Camera
#    X_cam = R * X_lidar + t
# ----------------------------
pts_lidar_h = pts_lidar.T           # shape = (3, N)
pts_cam_h   = (R @ pts_lidar_h) + t # shape = (3, N)
valid      = pts_cam_h[2, :] > 0    # 只保留 z>0 的点
pts_cam    = pts_cam_h[:, valid].T  # shape = (M,3)

# ----------------------------
# 4. 投影到图像平面
# ----------------------------
rvec = np.zeros((3,1), dtype=np.float64)  # 已在相机坐标系中
tvec = np.zeros((3,1), dtype=np.float64)

proj_pts, _ = cv2.projectPoints(
    objectPoints=pts_cam,
    rvec=rvec,
    tvec=tvec,
    cameraMatrix=camera_matrix,
    distCoeffs=dist_coeffs
)
proj_pts = proj_pts.reshape(-1,2).astype(int)

# ----------------------------
# 5. 在图像上画出投影点
# ----------------------------
vis_img = img.copy()
h, w    = img.shape[:2]
for (u, v) in proj_pts:
    if 0 <= u < w and 0 <= v < h:
        cv2.circle(vis_img, (u, v), 1, (0, 0, 0), -1)

# 显示或保存结果
cv2.namedWindow("LiDAR->Camera Projection", cv2.WINDOW_NORMAL)
cv2.imshow("LiDAR->Camera Projection", vis_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
# cv2.imwrite("projected_result.jpg", vis_img)
