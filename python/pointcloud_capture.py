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
from threading import Thread
from queue import Queue
from sensor_msgs.msg import PointCloud2, PointField  # Import PointField


# header: 
#   seq: 2715
#   stamp: 
#     secs: 115
#     nsecs: 254250000
#   frame_id: "D435_head_camera_color_optical_frame"
# height: 1
# width: 29472
# fields: 
#   - 
#     name: "x"
#     offset: 0
#     datatype: 7
#     count: 1
#   - 
#     name: "y"
#     offset: 4
#     datatype: 7
#     count: 1
#   - 
#     name: "z"
#     offset: 8
#     datatype: 7
#     count: 1
#   - 
#     name: "rgb"
#     offset: 16
#     datatype: 7
#     count: 1
# is_bigendian: False
# point_step: 32
# row_step: 0
# data:


class PointCloudRecorder:
    def __init__(self, save_path=None, compression='lz4', 
                 voxel_size=0.01, min_range=0.5, max_range=4.0, skip_frames=5):
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
        self.skip_frames = skip_frames
        self.frame_count = 0
        
        # Threaded processing
        self.queue = Queue()
        self.worker_thread = Thread(target=self.process_queue, daemon=True)
        self.worker_thread.start()
        
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
        """Push incoming point cloud messages to the processing queue."""
        self.frame_count += 1
        if self.frame_count % self.skip_frames == 0:
            rospy.loginfo(f"Enqueuing frame {self.frame_count} for processing...")
            self.queue.put(msg)
    
    def process_queue(self):
        """Threaded processing of point cloud messages."""
        while not rospy.is_shutdown():
            msg = self.queue.get()
            if msg is None:
                break
            
            try:
                start_time = time.time()

                # Convert PointCloud2 data to NumPy
                points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 8)  # x, y, z, rgb
                
                # Filter by range
                mask = (self.min_range < points[:, 2]) & (points[:, 2] < self.max_range)
                points = points[mask]

                # Apply voxel grid filtering
                cloud = pcl.PointCloud()
                cloud.from_array(points[:, :3].astype(np.float32))

                voxel_filter = cloud.make_voxel_grid_filter()
                voxel_filter.set_leaf_size(self.voxel_size, self.voxel_size, self.voxel_size)
                filtered_cloud = voxel_filter.filter()

                # Define the fields (x, y, z, rgb)
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1)
                ]

                # Create PointCloud2 message for filtered cloud
                filtered_msg = PointCloud2()
                filtered_msg.header = msg.header
                filtered_msg.height = 1
                filtered_msg.width = filtered_cloud.size
                filtered_msg.fields = fields
                filtered_msg.is_bigendian = False
                filtered_msg.point_step = 32  # 4 bytes per field (x, y, z, rgb)
                filtered_msg.row_step = filtered_cloud.size * filtered_msg.point_step
                filtered_msg.data = filtered_cloud.to_array().astype(np.float32).tobytes()


                # Save to bag
                self.bag.write('/D435_head_camera/depth/color/points', filtered_msg)

                rospy.loginfo(f"Processed frame {self.frame_count} in {time.time() - start_time:.2f}s")

            except Exception as e:
                rospy.logwarn(f"Error processing point cloud: {str(e)}")

    
    def stop_recording(self):
        """Stop processing and close the bag file."""
        self.queue.put(None)  # Signal the thread to stop
        self.worker_thread.join()  # Ensure all processing is done
        if self.bag:  # Ensure the bag is open before closing
            self.bag.close()
            print("Recording stopped. Bag file saved.")


if __name__ == '__main__':
    try:
        recorder = PointCloudRecorder(
            save_path='/home/wang/forest_ws/src/centauro_horizon/data',
            compression='lz4',
            voxel_size=0.05,   # Coarser downsampling
            min_range=0.5,
            max_range=4.0,
            skip_frames=5      # Process every 5th frame
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass