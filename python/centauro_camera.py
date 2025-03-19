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
from ultralytics import YOLO  # Import YOLO model from the ultralytics package
import os
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

frame_count = 0
bridge = CvBridge()
depth_image = None
rgb_image = None
camera_pose = None

save_folder = os.path.expanduser("~/room")
os.makedirs(save_folder, exist_ok=True)

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        print(f"Error converting depth image: {e}")

def image_callback(msg):
    global rgb_image
    try:
        rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        print(f"Error converting depth image: {e}")

def link_states_callback(msg: LinkStates):
    global camera_pose

    # Find the index of the camera link
    link_name = "realsense::D435i_camera_bottom_screw_frame"
    if link_name in msg.name:
        idx = msg.name.index(link_name)
        camera_pose_msg = msg.pose[idx]

        # Extract position
        position = camera_pose_msg.position

        # Extract orientation (quaternion)
        orientation = camera_pose_msg.orientation
        rotation_matrix = Rotation.from_quat([
            orientation.x, 
            orientation.y, 
            orientation.z, 
            orientation.w
        ]).as_matrix()

        # Create a 4x4 transformation matrix
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rotation_matrix
        pose_matrix[:3, 3] = [position.x, position.y, position.z]

        # Store the camera pose matrix
        camera_pose = pose_matrix
        # print("Camera Pose Matrix:\n", camera_pose)
    else:
        print("Camera link not found in link states.")

        
def record_information(frame_count):
    if depth_image is not None and camera_pose is not None:
        depth_image_path = os.path.join(save_folder, f"depth_image_{frame_count}.png")
        rgb_image_path = os.path.join(save_folder, f"rgb_image_{frame_count}.jpg")
        print("save_folder = ", save_folder)
        cv2.imwrite(depth_image_path, depth_image)
        cv2.imwrite(rgb_image_path, rgb_image)

        # Save the camera pose matrix to traj.txt
        traj_file_path = os.path.join(save_folder, "traj.txt")
        with open(traj_file_path, "a") as traj_file:
            # Flatten the camera pose matrix and write to the text file
            # traj_file.write(f"Frame {frame_count} Pose Matrix:\n")
            np.savetxt(traj_file, camera_pose.reshape(1, 16), fmt="%.6f")
            # traj_file.write("\n\n")
        
        print(f"Recorded depth image and camera pose for frame {frame_count}")
    else:
        print("Depth image or camera pose is not available yet.")
        

rospy.init_node('centauro_camera')
rospy.sleep(1.)    

rate = rospy.Rate(10)


rospy.Subscriber("/D435i_camera/aligned_depth_to_color/image_raw", Image, depth_callback) 
rospy.Subscriber("/D435i_camera/color/image_raw", Image, image_callback) 

rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)


while not rospy.is_shutdown():
    # Call the function to record information at each loop cycle
    record_information(frame_count)
    
    # Increment frame count
    frame_count += 1
    
    # Sleep for the rate duration
    rate.sleep()
