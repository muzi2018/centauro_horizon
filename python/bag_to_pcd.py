#!/usr/bin/python3

import rosbag
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np

# Input ROS bag file
bag_file = "../data/2025-03-01-19-23-43.bag"
topic_name = "/octomap_velodyne"
output_pcd = "output.pcd"

bag = rosbag.Bag(bag_file, "r")

for topic, msg, t in bag.read_messages(topics=[topic_name]):
    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    points = np.array(points)

    # Convert to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Save as PCD file
    o3d.io.write_point_cloud(output_pcd, pcd)
    print(f"Saved: {output_pcd}")
    break  # Remove this if you want to save multiple frames

bag.close()
