#!/usr/bin/env python3
# pcd2ply.py
# Usage:
#   python pcd2ply.py input.pcd                # 转单个文件
#   python pcd2ply.py /path/to/folder --glob "*.pcd"  # 批量转
#   python pcd2ply.py input.pcd -o output.ply  # 指定输出文件
# Requirements: pip install open3d>=0.17

import argparse, pathlib, sys, numpy as np
import open3d as o3d

def tpcd_to_legacy_with_intensity_colors(tpcd: o3d.t.geometry.PointCloud) -> o3d.geometry.PointCloud:
    # 如果没有颜色但有intensity，转成灰度颜色
    has_colors = "colors" in tpcd.point and tpcd.point["colors"].shape[1] == 3
    if (not has_colors) and ("intensity" in tpcd.point):
        inten = tpcd.point["intensity"].to(o3d.core.float32).cpu().numpy().reshape(-1, 1)
        # 归一化到[0,1]，避免全零或常值
        i_min, i_max = float(np.min(inten)), float(np.max(inten))
        if i_max > i_min:
            inten01 = (inten - i_min) / (i_max - i_min)
        else:
            inten01 = np.zeros_like(inten)
        colors = np.repeat(inten01, 3, axis=1)
        tpcd.point["colors"] = o3d.core.Tensor(colors, dtype=o3d.core.float32)
    return tpcd.to_legacy()

def convert_one(src: pathlib.Path, dst: pathlib.Path, ascii: bool = False):
    # 用Tensor版IO读，尽量保留属性
    try:
        tpcd = o3d.t.io.read_point_cloud(str(src))
        pcd = tpcd_to_legacy_with_intensity_colors(tpcd)
    except Exception:
        # 回退到legacy读
        pcd = o3d.io.read_point_cloud(str(src))
    if pcd.is_empty():
        raise RuntimeError(f"Read empty point cloud from {src}")
    # 保存为PLY（binary默认更小；需要ASCII可加 --ascii）
    o3d.io.write_point_cloud(str(dst), pcd, write_ascii=ascii)
    return pcd

def main():
    ap = argparse.ArgumentParser(description="Convert PCD to PLY with Open3D (supports batch).")
    ap.add_argument("input", help="PCD 文件或目录")
    ap.add_argument("-o","--output", help="输出PLY文件（仅当输入为单文件时使用）")
    ap.add_argument("--glob", default="*.pcd", help="目录批量模式的通配符，默认 *.pcd")
    ap.add_argument("--ascii", action="store_true", help="以ASCII方式写PLY（默认二进制）")
    args = ap.parse_args()

    in_path = pathlib.Path(args.input)
    if in_path.is_file():
        src = in_path
        if args.output:
            dst = pathlib.Path(args.output)
        else:
            dst = src.with_suffix(".ply")
        pcd = convert_one(src, dst, ascii=args.ascii)
        print(f"OK: {src.name} -> {dst.name} | points={np.asarray(pcd.points).shape[0]} "
              f"{'colors' if pcd.has_colors() else ''} {'normals' if pcd.has_normals() else ''}")
    elif in_path.is_dir():
        files = sorted(in_path.glob(args.glob))
        if not files:
            print(f"目录中未找到匹配 {args.glob} 的文件", file=sys.stderr); sys.exit(1)
        for src in files:
            dst = src.with_suffix(".ply")
            try:
                pcd = convert_one(src, dst, ascii=args.ascii)
                print(f"OK: {src.name} -> {dst.name} | N={np.asarray(pcd.points).shape[0]}")
            except Exception as e:
                print(f"FAIL: {src.name} | {e}", file=sys.stderr)
    else:
        print("输入路径不存在", file=sys.stderr); sys.exit(1)

if __name__ == "__main__":
    main()
