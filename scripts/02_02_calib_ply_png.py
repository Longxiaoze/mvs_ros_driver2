#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
interactive_color_ply.py  (updated)

依赖：
  pip install open3d pyyaml pillow numpy opencv-python matplotlib
"""

import yaml
import numpy as np
import open3d as o3d
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import sys
import os


def _parse_opencv_matrix(node):
    if isinstance(node, dict) and "data" in node and "rows" in node and "cols" in node:
        rows, cols = int(node["rows"]), int(node["cols"])
        return np.array(node["data"], dtype=float).reshape(rows, cols)
    elif isinstance(node, list):
        return np.array(node, dtype=float)
    return None

def _load_via_cvfs(yaml_path):
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f"OpenCV FileStorage 无法打开: {yaml_path}")

    # 优先本工具保存的键
    nodeK = fs.getNode("camera_matrix")
    nodeD = fs.getNode("distortion_coefficients")
    # 兼容其它常见键
    if nodeK.empty(): nodeK = fs.getNode("K")
    if nodeK.empty(): nodeK = fs.getNode("k")
    if nodeD.empty(): nodeD = fs.getNode("D")
    if nodeD.empty(): nodeD = fs.getNode("d")

    if nodeK.empty():
        fs.release()
        raise ValueError("在 YAML 中未找到 camera_matrix/K/k")
    K = nodeK.mat().astype(float)

    if nodeD.empty():
        dist = np.zeros((5,1), dtype=float)  # 没有就给 5 个 0
    else:
        d = nodeD.mat()
        dist = np.array(d, dtype=float).reshape(-1,1)

    fs.release()
    return K, dist

def load_camera_info(yaml_path):
    # 先尝试用 PyYAML（能解析 %YAML 1.0 / 1.2 这类头）
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        K, dist = None, None
        if isinstance(data, dict):
            if "camera_matrix" in data:
                K = _parse_opencv_matrix(data["camera_matrix"])
            if "distortion_coefficients" in data:
                d = _parse_opencv_matrix(data["distortion_coefficients"])
                if d is not None:
                    dist = np.array(d, dtype=float).reshape(-1,1)
            if K is None and "K" in data:
                K = np.array(data["K"], dtype=float).reshape(3,3)
            if dist is None and "D" in data:
                dist = np.array(data["D"], dtype=float).reshape(-1,1)
            if K is None and "k" in data:
                K = np.array(data["k"], dtype=float).reshape(3,3)
            if dist is None and "d" in data:
                dist = np.array(data["d"], dtype=float).reshape(-1,1)
        if K is not None and dist is not None:
            return K.astype(float), dist.astype(float)
        # 键不完整则转 FileStorage
        return _load_via_cvfs(yaml_path)
    except Exception:
        # 解析失败（大概率因为 %YAML:1.0 头），直接走 FileStorage
        return _load_via_cvfs(yaml_path)
def pick_image_points(img_path, num_points):
    img = np.array(Image.open(img_path))
    plt.figure("Image - pick {} points".format(num_points))
    plt.imshow(img)
    plt.axis('off')
    print(f"请在图像窗口中依次点击 {num_points} 个点，然后回车。")
    pts = plt.ginput(num_points, timeout=0)
    plt.close()
    return np.array(pts, dtype=float)  # shape (N,2)

def pick_pointcloud_points(ply_path, num_points):
    pcd = o3d.io.read_point_cloud(ply_path)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"PointCloud - pick {num_points} points")
    vis.add_geometry(pcd)
    print(f"请在点云窗口中按 Shift+左键 点击 {num_points} 个点，然后关闭窗口。")
    vis.run()
    idx = vis.get_picked_points()
    vis.destroy_window()
    if len(idx) != num_points:
        print(f"[WARN] 您选中了 {len(idx)} 个点，但需要 {num_points} 个。")
    return np.array(idx, dtype=int), np.asarray(pcd.points)

def solve_extrinsic(object_pts, image_pts, K, dist):
    """
    object_pts: (N,3) 3D lidar 点
    image_pts:  (N,2) 像素坐标
    K: (3,3)
    dist: (n,1) 畸变系数
    """
    success, rvec, tvec = cv2.solvePnP(
        object_pts, image_pts, K, dist,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not success:
        raise RuntimeError("solvePnP 失败")
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3,1)
    print("[INFO] 计算得到外参：")
    print("R =\n", R)
    print("t =\n", t.flatten())
    return R, t

def colorize_point_cloud(ply_path, img_path, K, dist, R_ex, t_ex):
    """
    使用 cv2.projectPoints 来考虑畸变对投影的影响。
    """
    pcd = o3d.io.read_point_cloud(ply_path)
    pts = np.asarray(pcd.points)  # (M,3)

    # LiDAR -> Camera
    pts_cam = (R_ex @ pts.T + t_ex).T  # (M,3)

    # 只投影 Z>0 的点
    zpos = pts_cam[:, 2] > 1e-6
    pts_cam_positive = pts_cam[zpos]

    # 用 projectPoints 投影（会自动使用 K 和 dist）
    rvec = np.zeros((3,1), dtype=float)  # 已在相机坐标系，无需额外旋转/平移
    tvec = np.zeros((3,1), dtype=float)
    imgpoints, _ = cv2.projectPoints(pts_cam_positive, rvec, tvec, K, dist)
    imgpoints = imgpoints.reshape(-1, 2)  # (m,2)

    img = np.array(Image.open(img_path))
    H, W = img.shape[:2]

    colors = np.zeros_like(pts, dtype=float)
    u = imgpoints[:, 0]
    v = imgpoints[:, 1]

    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    ui = u[valid].astype(int)
    vi = v[valid].astype(int)

    # 把 valid 对应回原数组索引
    idx_positive = np.where(zpos)[0]
    idx_valid_global = idx_positive[valid]

    colors[idx_valid_global] = img[vi, ui] / 255.0
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def main():
    if len(sys.argv)!=5:
        print("用法: python3 interactive_color_ply.py <calib_yaml> <pointcloud.ply> <image.png> <num_points>")
        sys.exit(1)

    yaml_path, ply_path, img_path, n_str = sys.argv[1:]
    if not all(os.path.isfile(p) for p in (yaml_path, ply_path, img_path)):
        print("错误：文件不存在")
        sys.exit(1)
    N = int(n_str)

    # 1. 读取内参 + 畸变
    K, dist = load_camera_info(yaml_path)
    print("[INFO] 内参 K =\n", K)
    print("[INFO] 畸变 dist ({}x1) = {}".format(len(dist), dist.ravel()))

    # 2. 图像交互点击
    img_pts = pick_image_points(img_path, N)

    # 3. 点云交互点击
    idxs, pts_world = pick_pointcloud_points(ply_path, N)
    obj_pts = pts_world[idxs]

    # 4. 计算外参
    R_ex, t_ex = solve_extrinsic(obj_pts, img_pts, K, dist)

    # 5. 上色并可视化（考虑畸变）
    colored_pcd = colorize_point_cloud(ply_path, img_path, K, dist, R_ex, t_ex)

    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    cam_frame   = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    T = np.eye(4)
    T[:3,:3], T[:3,3] = R_ex, t_ex.flatten()
    cam_frame.transform(T)
    o3d.visualization.draw_geometries([colored_pcd, world_frame, cam_frame],
                                      window_name="Colored Point Cloud",
                                      width=1024, height=768)

if __name__ == "__main__":
    main()

