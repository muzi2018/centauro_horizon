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

class PointCloudRecorder:
    def __init__(self, save_path=None, compression='lz4'):
        # Initialize ROS node
        rospy.init_node('point_cloud_recorder')
        self.compression = compression  # This line sets the attribute

        # Set default save path if none provided
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
        self.bag = rosbag.Bag(self.bag_path, 'w')
        
        # Subscribe to point cloud topic
        self.subscriber = rospy.Subscriber(
            '/D435_head_camera/depth/color/points',
            PointCloud2,
            self.callback
        )
    
    def callback(self, msg):
        # Write the message to bag file with compression
        self.bag.write('/D435_head_camera/depth/color/points', msg)
    
    def start_recording(self, rate=2):  # Default to 2 Hz for mapping
        print("Recording point cloud data for mapping...")
        try:
            self.rate = rospy.Rate(rate)  # Adjust recording rate
            while not rospy.is_shutdown():
                self.rate.sleep()
        except KeyboardInterrupt:
            print("\nStopping recording...")
        finally:
            self.stop_recording()
    
    def stop_recording(self):
        # Close the bag file
        self.bag.close()
        print("Recording stopped. Bag file saved.")

if __name__ == '__main__':
    try:
        # Pass compression parameter when creating recorder
        recorder = PointCloudRecorder(
            '/home/wang/forest_ws/src/centauro_horizon/data',
            compression='lz4'
        )
        recorder.start_recording(rate=2)
    except rospy.ROSInterruptException:
        pass