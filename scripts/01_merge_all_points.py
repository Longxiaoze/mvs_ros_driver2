import os
import open3d as o3d
import argparse

def merge_pcd_folder(input_folder, output_filename="merged.ply"):
    # 获取所有pcd文件
    pcd_files = [f for f in os.listdir(input_folder) if f.endswith(".pcd") or f.endswith(".ply")]
    if not pcd_files:
        print(f"No .pcd files found in {input_folder}")
        return

    print(f"Found {len(pcd_files)} .pcd files, start merging...")

    merged_pcd = o3d.geometry.PointCloud()

    for f in pcd_files:
        file_path = os.path.join(input_folder, f)
        pcd = o3d.io.read_point_cloud(file_path)
        print(f"Read {file_path}, points: {len(pcd.points)}")
        merged_pcd += pcd  # 点云相加即合并

    print(f"Merged point cloud total points: {len(merged_pcd.points)}")

    output_path = os.path.join(input_folder, output_filename)
    o3d.io.write_point_cloud(output_path, merged_pcd)
    print(f"Saved merged point cloud to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge all PCD files in a folder into one PCD")
    parser.add_argument("input_folder", type=str, help="Folder path containing .pcd files to merge")
    parser.add_argument("--output", type=str, default="merged.ply", help="Output merged pcd filename")
    args = parser.parse_args()

    merge_pcd_folder(args.input_folder, args.output)
