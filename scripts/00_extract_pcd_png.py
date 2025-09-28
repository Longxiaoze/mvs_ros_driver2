#!/usr/bin/env python3
import os

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2, Image
from rclpy.serialization import deserialize_message
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
import cv2

def main():
    # --- 配置区域 ---
    bag_path = '/media/longxiaoze/Data1/aist/perspective/test_time_20250928/test_time_20250928_0.db3'
    output_dir = './extracted2'
    image_topic = '/front_camera/image_raw'
    cloud_topic = '/livox/lidar'
    os.makedirs(os.path.join(output_dir, 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'pointclouds'), exist_ok=True)
    # ----------------

    # 打开 rosbag
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    bridge = CvBridge()

    # 逐条读取
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # Image
        if topic == image_topic:
            msg: Image = deserialize_message(data, Image)
            # 转成 OpenCV BGR8
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
            fn = os.path.join(output_dir, 'images', f"{ts}.png")
            cv2.imwrite(fn, cv_img)
            print(f"Saved image: {fn}")

        # PointCloud2
        elif topic == cloud_topic:
            msg: PointCloud2 = deserialize_message(data, PointCloud2)
            # 读 XYZ
            points = [ [x, y, z]
                       for x, y, z in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True) ]
            if not points:
                continue
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(np.array(points))
            ts = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
            fn = os.path.join(output_dir, 'pointclouds', f"{ts}.ply")
            o3d.io.write_point_cloud(fn, pc)
            print(f"Saved point cloud: {fn}")

if __name__ == '__main__':
    main()
