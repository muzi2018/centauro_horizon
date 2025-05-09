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
from ultralytics import SAM
import pygame
from std_msgs.msg import String
import json

update_flag = True
frame = None
depth_frame = None

focal_length_x = 924.2759399414062
focal_length_y = 924.2759399414062

center_x = 640.0
center_y = 360.0

intrinsic_matrix = np.array([[focal_length_x, 0, center_x],                         # Replace with actual values
                            [0, focal_length_y, center_y],
                            [0, 0, 1]])
confidence_threshold = 0.5  


def pixel_to_3d(cx, cy, depth, intrinsic_matrix):
    # Extract camera intrinsic parameters
    fx = intrinsic_matrix[0, 0]     # Focal length in x
    fy = intrinsic_matrix[1, 1]     # Focal length in y
    cx_ = intrinsic_matrix[0, 2]    # Principal point x
    cy_ = intrinsic_matrix[1, 2]    # Principal point y
    
    # Convert 2D pixel to 3D coordinates
    Z = depth  # depth in meters
    # print("depth type: ", type(Z))
    X = (cx - cx_) * Z / fx
    Y = (cy - cy_) * Z / fy
    # print("X: ", X)
    # print("Y: ", Y)
    
    return X, Y, Z

def get_depth_at(x, y):
    """Get depth value at a given (x, y) coordinate."""
    global depth_frame
    if depth_frame is None:
        return None
    h, w = depth_frame.shape
    x, y = int(x), int(y)
    if 0 <= x < w and 0 <= y < h:
        return depth_frame[y, x] * 0.001  
    return None

def depth_callback(msg):
    global depth_frame
    try:
        depth_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  # Depth in millimeters
    except Exception as e:
        print(f"Error converting depth image: {e}")



def detect_edges(image, x1, y1, x2, y2):
    """Detect edges within the bounding box using Canny edge detection."""
    # Crop the region of interest (ROI) for the chair from the image
    roi = image[y1:y2, x1:x2]

    # Convert to grayscale for edge detection
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    edges = cv2.Canny(gray, 100, 200)
    # print("edges :", edges)

    # Return the edge-detected image
    return edges


def choose_point_on_edge(edges, x1, y1):
    global depth_frame
    edge_points = np.argwhere(edges > 0)  # Get coordinates of edge points (non-zero values)
    # print("edge_points: ", edge_points)
    if edge_points.size == 0:
        return None
    
    X1, Y1, _ = pixel_to_3d(x1, y1, 0, intrinsic_matrix)
    # print("size = ", edge_points.size)
    # For each edge point, calculate the depth
    for point in edge_points:
        edge_y, edge_x = point
        depth = get_depth_at(edge_x + x1, edge_y + y1)  # Get depth at the edge point
        Edge_x, Edge_y, _ = pixel_to_3d(edge_x, edge_y, depth, intrinsic_matrix)

        if depth is not None and depth < 4.5 and Edge_y <= Y1 :
            # Return the point (x, y) in the image coordinates
            # print("Edge_y = ", Edge_y)
            # print("Y1 = ", Y1)
            
            return (edge_x + x1, edge_y + y1, depth)

    return None  # Return None if no valid point found


edge_x = 0
edge_y = 0
X = 0
Y = 0
Z = 0
def image_callback(msg):
    global frame, obj_dict, edge_x, edge_y, X, Y, Z
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        frame = frame.copy() 
        frame_vis = np.flip(frame, axis=2)                  # Convert from BGR to RGB if necessary
        frame_vis = np.transpose(frame_vis, (1, 0, 2))      # Convert from (H, W, C) to (W, H, C)
        surface = pygame.surfarray.make_surface(frame_vis)  # Convert to Pygame surface
        screen.blit(surface, (0, 0))                        # Display the image   
    except Exception as e:
        print(f"Error converting image: {e}")
        return

    cnt = 0
    results_det = model_det(frame, verbose=False)                   # YOLO detection        
    detections = results_det[0]                                     # Get the first (and usually only) result from the list

    boxes = detections.boxes                                        # Bounding boxes in format (x1, y1, x2, y2)
    probs = detections.probs                                        # Confidence scores for each detection

  
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()                                           # Convert box tensor to list of coordinates
        bbox_center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
        
        conf = box.conf[0].item()
        cls = int(box.cls[0].item())
        class_name = model_det.names[cls]

        # if conf < confidence_threshold:
        #     continue                                                                    # Skip this detection
 
        # depth = get_depth_at(bbox_center[0], bbox_center[1])
        # X, Y, Z = pixel_to_3d(bbox_center[0], bbox_center[1], depth, intrinsic_matrix)  # Convert to 3D     
        
        # Apply edge detection on the detected chair region
        edges = detect_edges(frame, int(x1), int(y1), int(x2), int(y2))
        # Choose a point on the edge with depth < 4.5 meters
        point = choose_point_on_edge(edges, int(x1), int(y1))
        if point is not None:
            edge_x, edge_y, depth = point
            
            depth = get_depth_at(edge_x, edge_y)
            X, Y, Z = pixel_to_3d(edge_x, edge_y, depth, intrinsic_matrix)  # Convert to 3D 
            
            # print("edge point is not none ...")
            pygame.draw.circle(screen, (0, 255, 0), (int(edge_x), int(edge_y)), 5)  # Draw the selected point in green 
            # print(f"Object: {class_name}")
            font = pygame.font.Font(None, 36)                                                               # Create a font object (None means default font)
            text_surface = font.render(f"{class_name} ({X:.2f}, {Y:.2f}, {Z:.2f})", True, (255, 255, 255))  # Render the text with XYZ
            screen.blit(text_surface, (x1, y1 - 40))                                                        # Position the text just above the bounding box (adjust the offset as needed)
            pygame.draw.rect(screen, RED, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)), 5)                # Draw a rectangle 
    
            up_left_depth = get_depth_at(x1, y1)
            up_left_X, up_left_Y, up_left_Z = pixel_to_3d(x1, y1, up_left_depth, intrinsic_matrix)  # Convert to 3D
            
            down_right_depth = get_depth_at(x2, y2)
            down_right_X, down_right_Y, down_right_Z = pixel_to_3d(x2, y2, down_right_depth, intrinsic_matrix)  # Convert to 3D '
            
            pygame.draw.circle(screen, (0, 255, 0), (int(x1), int(y1)), 5)  # Draw the selected point in green 
            text_surface = font.render(f"up_left:  ({up_left_X:.2f}, {up_left_Y:.2f}, {up_left_Z:.2f})", True, (255, 255, 255))  # Render the text with XYZ
            screen.blit(text_surface, (x1, y1))                                                        # Position the text just 
            
            pygame.draw.circle(screen, (0, 255, 0), (int(x2), int(y2)), 5)  # Draw the selected point in green 
            text_surface = font.render(f"down_right: ({down_right_X:.2f}, {down_right_Y:.2f}, {down_right_Z:.2f})", True, (255, 255, 255))  # Render the text with XYZ
            screen.blit(text_surface, (x2, y2))                                                        # Position the text just 
            
     
        if class_name == "chair" and cnt == 0:
            if "chair" not in obj_dict:
                obj_dict["chair"] = {"position": (0.0, 0.0, 0.0), "detected": False}
            obj_dict["chair"]["position"] = (X, Y, Z)
            obj_dict["chair"]["detected"] = True

        cnt = cnt + 1



        
        # pygame.draw.circle(screen, (255, 0, 0), (int(bbox_center[0]), int(bbox_center[1])), 5)          # Draw red point

    pygame.display.update()
        
    if boxes is None or len(boxes) == 0:

        obj_dict = {}
        print('boxes is None')
        return

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
rgb_img_list = []
depth_img_list = []
def update_list(img_list, new_frame):
    """ Maintains only the last two frames in the list """
    img_list.append(new_frame)
    if len(img_list) > 2:
        img_list.pop(0)



bridge = CvBridge()
model_det = YOLO('yolo12n.pt') 
obj_dict = {}

pygame.init()
screen = pygame.display.set_mode((1280, 720))
pygame.display.set_caption('Detection')

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

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

rate = rospy.Rate(60)

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
pub_pos = rospy.Publisher('object_positions', String, queue_size=10)



while not rospy.is_shutdown():
    msg = json.dumps(obj_dict)
    pub_pos.publish(msg)


    rate.sleep()


