import os
import glob
import argparse
import cv2
import numpy as np

def collect_images(img_dir):
    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tiff", "*.tif")
    paths = []
    for e in exts:
        paths.extend(glob.glob(os.path.join(img_dir, e)))
    paths = sorted(paths)
    return paths

def save_yaml(fname, K, dist, rms, img_size, per_view_errors):
    fs = cv2.FileStorage(fname, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", K)
    fs.write("distortion_coefficients", dist)
    fs.write("rms_reprojection_error", float(rms))
    fs.write("image_width", int(img_size[0]))
    fs.write("image_height", int(img_size[1]))
    fs.write("per_view_errors", np.array(per_view_errors, dtype=np.float32))
    fs.release()

def main():
    parser = argparse.ArgumentParser(description="Calibrate camera from a folder of chessboard images.")
    parser.add_argument("--img_dir", required=True, help="Folder containing images.")
    parser.add_argument("--cols", type=int, required=True, help="Number of inner corners per row (columns).")
    parser.add_argument("--rows", type=int, required=True, help="Number of inner corners per column (rows).")
    parser.add_argument("--square", type=float, default=1.0, help="Square size in meters (or any unit).")
    parser.add_argument("--out_dir", default="annotated", help="Directory to save drawn corners.")
    parser.add_argument("--save", default="calib_results.yaml", help="YAML file to save calibration results.")
    parser.add_argument("--max_imgs", type=int, default=0, help="Use at most N images (0 = all).")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # 准备棋盘格 3D 物点（Z=0 的平面上）
    chess_size = (args.cols, args.rows)  # (columns, rows) 的“内角点”尺寸
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square  # 尺度

    objpoints = []  # 3D 点
    imgpoints = []  # 2D 角点
    used_images = []

    # 读取图片
    img_paths = collect_images(args.img_dir)
    if args.max_imgs > 0:
        img_paths = img_paths[:args.max_imgs]

    if len(img_paths) == 0:
        print("No images found in:", args.img_dir)
        return

    print(f"Found {len(img_paths)} images. Detecting corners ...")
    gray_shape = None

    # 角点检测参数
    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    # 也可以加 FAST_CHECK 加速，但可能漏检： | cv2.CALIB_CB_FAST_CHECK

    for i, p in enumerate(img_paths):
        img = cv2.imread(p)
        if img is None:
            print(f"[Skip] Cannot read: {p}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]  # (w,h)

        ret, corners = cv2.findChessboardCorners(gray, chess_size, flags=find_flags)
        if ret:
            # 亚像素优化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp.copy())
            imgpoints.append(corners)

            # 绘制可视化并保存
            vis = img.copy()
            cv2.drawChessboardCorners(vis, chess_size, corners, ret)
            base = os.path.splitext(os.path.basename(p))[0]
            out_path = os.path.join(args.out_dir, f"{base}_corners.jpg")
            cv2.imwrite(out_path, vis)
            used_images.append(p)
            print(f"[OK] {p} -> corners saved to {out_path}")
        else:
            print(f"[Fail] Chessboard not found: {p}")

    if len(objpoints) < 3:
        print("Not enough valid detections for calibration (<3). Aborting.")
        return

    # 相机标定
    print("\nCalibrating ...")
    # 注意：如果画面没有横纵比畸变太大，可以使用 CALIB_RATIONAL_MODEL 提高精度
    calib_flags = 0
    # calib_flags |= cv2.CALIB_RATIONAL_MODEL
    # 若已知主点在图像中心，可加： calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_shape, None, None, flags=calib_flags
    )
    print(f"RMS reprojection error: {rms:.6f}")
    print("Camera matrix (K):\n", K)
    print("Distortion coeffs:\n", dist.ravel())

    # 每张图像的重投影误差
    per_view_errors = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        per_view_errors.append(err)

    # 保存结果
    save_yaml(args.save, K, dist, rms, gray_shape, per_view_errors)
    print(f"\nSaved calibration to: {args.save}")
    print(f"Used {len(used_images)} images. Visualizations in: {args.out_dir}")

    # 额外：示例去畸变一张图
    sample = cv2.imread(used_images[0])
    h, w = sample.shape[:2]
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0)  # alpha=0 更强去畸变裁切
    undist = cv2.undistort(sample, K, dist, None, newK)
    undist_path = os.path.join(args.out_dir, "undist_sample.jpg")
    cv2.imwrite(undist_path, undist)
    print(f"Undistorted sample saved to: {undist_path}")

if __name__ == "__main__":
    main()

# python calib_chessboard_folder.py \
#   --img_dir /path/to/images \
#   --cols 9 --rows 6 \           # 内角点数量（列×行），注意是“内角点”，不是格子数
#   --square 0.025 \              # 单位米；若只关心像素内参，可设为 1.0
#   --out_dir calib_vis \         # 角点可视化输出目录
#   --save calib_results.yaml     # 标定结果文件