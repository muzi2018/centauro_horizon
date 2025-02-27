#!/usr/bin/python3
import rospy
import rospkg
import numpy as np
from std_msgs.msg import Float64
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from geometry_msgs.msg import Wrench
from scipy.spatial.transform import Rotation
import cartesian_interface.roscpp_utils as roscpp
import cartesian_interface.pyci as pyci
import cartesian_interface.affine3
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import colorama
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import PointCloud2
import rosbag
import time
import os
import pcl

class PointCloudRecorder:
    def __init__(self, save_path=None, compression='lz4', 
                 voxel_size=0.01, min_range=0.5, max_range=4.0):
        """
        Initialize the PointCloudRecorder with compression and filtering options.
        
        Args:
            save_path (str): Directory to save the bag file
            compression (str): Compression algorithm ('lz4' or 'zstd')
            voxel_size (float): Size of voxel grid for downsampling
            min_range (float): Minimum distance to keep points
            max_range (float): Maximum distance to keep points
        """
        rospy.init_node('point_cloud_recorder')
        
        # Compression settings
        # Compression settings
        self.compression = compression
        
        # If zstd compression is requested, fall back to bz2 since zstd isn't supported
        if compression == 'zstd':
            self.compression = 'bz2'
        
        # Ensure compression is one of the supported values
        if self.compression not in ['none', 'bz2', 'lz4']:
            rospy.logwarn(f"Unsupported compression '{self.compression}', defaulting to 'none'")
            self.compression = 'none'
        if save_path is None:
            save_path = os.path.join(os.path.expanduser('~'), 'rosbags')
        
        # Create directory if it doesn't exist
        os.makedirs(save_path, exist_ok=True)
        
        # Create bag file with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        bag_filename = f'pointcloud_data_{timestamp}.bag'
        self.bag_path = os.path.join(save_path, bag_filename)
        print(f"Saving bag file to: {self.bag_path}")
        
        # Create bag file
        self.bag = rosbag.Bag(self.bag_path, 'w', compression=self.compression)
        
        # Initialize filters
        self.voxel_size = voxel_size
        self.min_range = min_range
        self.max_range = max_range
        
        # Subscribe to point cloud topic
        self.subscriber = rospy.Subscriber(
            '/D435_head_camera/depth/color/points',
            PointCloud2,
            self.callback
        )
    
    def filter_points(self, points):
        """
        Filter points based on distance range.
        
        Args:
            points (numpy.array): Points array (x,y,z,rgb)
            
        Returns:
            numpy.array: Filtered points
        """
        mask = (points[:, 2] > self.min_range) & (points[:, 2] < self.max_range)
        return points[mask]
    
    def callback(self, msg):
        """
        Process and write point cloud message to bag file.
        
        Args:
            msg (PointCloud2): Input point cloud message
        """
        try:
            # Convert to numpy array for processing
            points = np.frombuffer(msg.data, dtype=np.float32)
            points = points.reshape(-1, 8)  # x,y,z,rgb
            
            # Apply point range filtering
            filtered_points = self.filter_points(points)
            
            # Apply voxel grid filtering using PCL
            # Convert to PCL point cloud
            cloud = pcl.PointCloud()
            cloud.from_array(filtered_points[:, :3].astype(np.float32))

            # Create and apply voxel grid filter
            voxel_filter = pcl.VoxelGridFilter()
            voxel_filter.set_input_cloud(cloud)
            voxel_filter.set_leaf_size(self.voxel_size, self.voxel_size, self.voxel_size)
            filtered_cloud = voxel_filter.filter()

            # Convert back to numpy array
            voxel_points = np.zeros((filtered_cloud.size, 8), dtype=np.float32)
            voxel_points[:, :3] = np.array(filtered_cloud.to_array())

            # Copy RGB and other data from original points where possible
            # This is a simple approach - may need refinement based on specific needs
            for i, point in enumerate(voxel_points):
                # Find closest point in original data to preserve color information
                distances = np.sum((filtered_points[:, :3] - point[:3])**2, axis=1)
                closest_idx = np.argmin(distances)
                voxel_points[i, 3:] = filtered_points[closest_idx, 3:]
            
            # Create new message with filtered points
            filtered_msg = PointCloud2()
            filtered_msg.header = msg.header
            filtered_msg.height = 1
            filtered_msg.width = len(voxel_points)
            filtered_msg.fields = msg.fields
            filtered_msg.point_step = msg.point_step
            filtered_msg.data = voxel_points.tobytes()
            filtered_msg.is_bigendian = msg.is_bigendian
            filtered_msg.is_dense = msg.is_dense
            
            # Write to bag file with compression
            self.bag.write('/D435_head_camera/depth/color/points',
                        filtered_msg)
            
        except Exception as e:
            rospy.logwarn(f"Error processing point cloud: {str(e)}")
    
    def start_recording(self, rate=1):
        """
        Start recording point cloud data.
        
        Args:
            rate (int): Recording rate in Hz
        """
        print("Recording point cloud data...")
        try:
            self.rate = rospy.Rate(rate)
            while not rospy.is_shutdown():
                self.rate.sleep()
        except KeyboardInterrupt:
            print("\nStopping recording...")
        finally:
            self.stop_recording()
    
    def stop_recording(self):
        """Close the bag file and clean up."""
        self.bag.close()
        print("Recording stopped. Bag file saved.")

if __name__ == '__main__':
    try:
        # Create recorder with optimized settings
        recorder = PointCloudRecorder(
            '/home/wang/forest_ws/src/centauro_horizon/data',
            compression='lz4',
            voxel_size=0.01,
            min_range=0.5,
            max_range=4.0
        )
        recorder.start_recording(rate=1)
    except rospy.ROSInterruptException:
        pass