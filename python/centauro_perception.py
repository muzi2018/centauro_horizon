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
from ultralytics import SAM
# from pcdet.config import cfg, cfg_from_yaml_file
# from pcdet.models import build_network
# from pcdet.datasets.kitti.kitti_dataset import KittiDataset
import pygame
import os
from gazebo_msgs.msg import LinkStates


save_folder = os.path.expanduser("~/room")
os.makedirs(save_folder, exist_ok=True)
save_image_folder = os.path.expanduser("~/room/results")

frame = None
depth_frame = None
camera_pose = None
def compute_mask_center(mask_points):
    """Calculate the center of the mask"""
    if len(mask_points) == 0:
        return None  

    mask_points = np.array(mask_points, dtype=np.int32)

    moments = cv2.moments(mask_points)
    if moments['m00'] == 0:  # Avoid division by zero
        return None

    # Calculate the center of mass (centroid)
    center_x = int(moments['m10'] / moments['m00'])
    center_y = int(moments['m01'] / moments['m00'])
    
    return center_x, center_y


def pixel_to_3d(cx, cy, depth, intrinsic_matrix):
    # Extract camera intrinsic parameters
    fx = intrinsic_matrix[0, 0]  # Focal length in x
    fy = intrinsic_matrix[1, 1]  # Focal length in y
    cx_ = intrinsic_matrix[0, 2]  # Principal point x
    cy_ = intrinsic_matrix[1, 2]  # Principal point y
    
    # Convert 2D pixel to 3D coordinates
    Z = depth  # depth in meters
    X = (cx - cx_) * Z / fx
    Y = (cy - cy_) * Z / fy
    
    return X, Y, Z

# get intrinsic_matrix from camera driver, 

def link_states_callback(msg: LinkStates):
    global camera_pose

    # Find the index of the camera link
    link_name = "centauro::d435_head_motor"
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
        
def depth_callback(msg):
    global depth_frame
    try:
        depth_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  # Depth in millimeters
    except Exception as e:
        print(f"Error converting depth image: {e}")

def get_depth_at(x, y):
    """Get depth value at a given (x, y) coordinate."""
    global depth_frame
    if depth_frame is None:
        return None

    # Ensure x, y are within bounds
    h, w = depth_frame.shape
    x, y = int(x), int(y)
    if 0 <= x < w and 0 <= y < h:
        return depth_frame[y, x] * 0.001  # Convert from mm to meters
    return None
        
bridge = CvBridge()
# Load YOLO model
model_det = YOLO('yolo12n.pt')  # You can change the model to another pre-trained model (e.g., yolov8s.pt)

obj_dict = {}

# pygame.init()
# screen = pygame.display.set_mode((1280, 720))
# pygame.display.set_caption('Detection')
# Set up colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

def image_callback(msg):
    global frame
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # print("#####-----Converted image successfully-----#####")
    except Exception as e:
        print(f"Error converting image: {e}")
        return


def record_information(frame_count):
    if depth_frame is not None and camera_pose is not None:
        depth_image_path = os.path.join(save_image_folder, f"depth_image_{frame_count}.png")
        rgb_image_path = os.path.join(save_image_folder, f"rgb_image_{frame_count}.jpg")
        cv2.imwrite(depth_image_path, depth_frame)
        cv2.imwrite(rgb_image_path, frame)

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
                
def imu_callback(msg: Imu):
    global base_pose
    base_pose = np.zeros(7)
    base_pose[3:] = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def gt_twist_callback(msg):
    global base_twist
    base_twist = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                           msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
    
    
def gt_pose_callback(msg):
    global base_pose
    base_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                          msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                          msg.pose.orientation.w])
    



rospy.init_node('centauro_walk_srbd')
rospy.sleep(1.)    
'''
Load urdf and srdf
'''
urdf = rospy.get_param(param_name='/robot_description', default='')
if urdf == '':
    raise print('urdf not set')
srdf = rospy.get_param(param_name='/robot_description_semantic', default='')
if srdf == '':
    raise print('srdf not set')
file_dir = rospkg.RosPack().get_path('centauro_horizon')

rate = rospy.Rate(6)

'''
Build ModelInterface and RobotStatePublisher
'''
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

'''
xbot
'''
xbot_param = rospy.get_param(param_name="~xbot", default=False)

'''
open/closed loop
'''
closed_loop = rospy.get_param(param_name='~closed_loop', default=False)

if xbot_param:
    print ("xbot i on")
    # exit()
    robot = xbot.RobotInterface(cfg)
    model_fk = robot.model()
    robot.sense()

    if not closed_loop:
        rospy.Subscriber('/xbotcore/imu/imu_link', Imu, imu_callback)
        while base_pose is None:
            rospy.sleep(0.01)

        base_pose[0:3] = [0.07, 0., 0.8]
        base_twist = np.zeros(6)
    else:
        # rospy.Subscriber('/xbotcore/link_state/pelvis/pose', PoseStamped, gt_pose_callback)
        # rospy.Subscriber('/xbotcore/link_state/pelvis/twist', TwistStamped, gt_twist_callback)
        rospy.Subscriber('/centauro_base_estimation/base_link/pose', PoseStamped, gt_pose_callback)
        rospy.Subscriber('/centauro_base_estimation/base_link/twist', TwistStamped, gt_twist_callback)

        while base_pose is None or base_twist is None:
            rospy.sleep(0.01)

    q_init = robot.getJointPosition()
    q_init = robot.eigenToMap(q_init)

else:
    print('RobotInterface not created')

    q_init = {
        # 'torso_yaw': 0.00,  # 0.00,
        # 'j_arm1_1': 1.50,  # 1.60,
        # 'j_arm1_2': 0.1,  # 0.,
        # 'j_arm1_3': 0.2,  # 1.5,
        # 'j_arm1_4': -2.2,  # 0.3,
        # 'j_arm1_5': 0.00,  # 0.00,
        # 'j_arm1_6': -1.3,  # 0.,
        # 'j_arm1_7': 0.0,    # 0.0,

        # 'j_arm2_1': 1.50,  # 1.60,
        # 'j_arm2_2': 0.1,  # 0.,
        # 'j_arm2_3': -0.2,  # 1.5,
        # 'j_arm2_4': -2.2,  # -0.3,
        # 'j_arm2_5': 0.0,  # 0.0,
        # 'j_arm2_6': -1.3,  # 0.,
        # 'j_arm2_7': 0.0,    # 0.0,

        # 'd435_head_joint': 0.0,
        # 'velodyne_joint': 0.0,
        'hip_yaw_1': -0.746,
        'hip_pitch_1': -1.254,
        'knee_pitch_1': -1.555,
        'ankle_pitch_1': -0.3,

        'hip_yaw_2': 0.746,
        'hip_pitch_2': 1.254,
        'knee_pitch_2': 1.555,
        'ankle_pitch_2': 0.3,

        'hip_yaw_3': 0.746,
        'hip_pitch_3': 1.254,
        'knee_pitch_3': 1.555,
        'ankle_pitch_3': 0.3,

        'hip_yaw_4': -0.746,
        'hip_pitch_4': -1.254,
        'knee_pitch_4': -1.555,
        'ankle_pitch_4': -0.3,
    }
    base_pose = np.array([0.07, 0., 0.8, 0., 0., 0., 1.])
    base_twist = np.zeros(6)
    

rospy.Subscriber("/D435_head_camera/color/image_raw", Image, image_callback) 
rospy.Subscriber("/D435_head_camera/aligned_depth_to_color/image_raw", Image, depth_callback) 

rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)
frame_count = 0
while not rospy.is_shutdown():
    # Call the function to record information at each loop cycle
    record_information(frame_count)
    
    # Increment frame count
    frame_count += 1
    
    # Sleep for the rate duration
    rate.sleep()


