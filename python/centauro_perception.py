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
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from nav_msgs.msg import OccupancyGrid



update_flag = True
frame = None
depth_frame = None
def pixel_to_3d(cx, cy, depth, intrinsic_matrix):
    # Extract camera intrinsic parameters
    fx = intrinsic_matrix[0, 0]     # Focal length in x
    fy = intrinsic_matrix[1, 1]     # Focal length in y
    cx_ = intrinsic_matrix[0, 2]    # Principal point x
    cy_ = intrinsic_matrix[1, 2]    # Principal point y
    
    # Convert 2D pixel to 3D coordinates
    Z = depth  # depth in meters
    X = (cx - cx_) * Z / fx
    Y = (cy - cy_) * Z / fy
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


def detect_edges(image, x1, y1, x2, y2, mask=None, sam_mask=None):
    """Detect edges within the bounding box using Canny edge detection.
    Args:
        image: Input image
        x1, y1, x2, y2: Bounding box coordinates
        mask: Optional binary mask
        sam_mask: Optional SAM segmentation mask
    Returns:
        edges: Binary edge map
    """
    # Validate input image
    if image is None or not isinstance(image, np.ndarray):
        return None
    
    # Crop the region of interest (ROI) from the image
    try:
        roi = image[y1:y2, x1:x2]
    except IndexError:
        print(f"Invalid ROI coordinates: ({x1}, {y1}, {x2}, {y2})")
        return None
    
    if roi.size == 0:
        return None
    
    # Convert to grayscale for edge detection
    try:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    except cv2.error as e:
        print(f"Error converting to grayscale: {e}")
        return None
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Detect edges using Canny edge detection
    edges = cv2.Canny(blurred, 100, 200)
    
    # Apply masks if provided
    if mask is not None:
        # Ensure mask is the same size as edges
        resized_mask = cv2.resize(mask, (edges.shape[1], edges.shape[0]))
        edges = cv2.bitwise_and(edges, edges, mask=resized_mask)
    
    if sam_mask is not None:
        # Ensure SAM mask is in correct format
        if sam_mask is not None:
            sam_mask = np.array(sam_mask, dtype=np.float32)
            # Normalize SAM mask to [0,1] range
            sam_mask = sam_mask / sam_mask.max() if sam_mask.max() > 0 else sam_mask
            # Convert to binary mask
            _, sam_mask = cv2.threshold(sam_mask, 0.5, 255, cv2.THRESH_BINARY)
            sam_mask = sam_mask.astype(np.uint8)
            
            # Ensure SAM mask is the same size as edges
            resized_sam_mask = cv2.resize(sam_mask, (edges.shape[1], edges.shape[0]))
            edges = cv2.bitwise_and(edges, edges, mask=resized_sam_mask)
    
    return edges

def create_chair_mask(frame, x1, y1, x2, y2):
    """Create a mask for the chair region using YOLO detection."""
    # Crop the chair region
    chair_roi = frame[y1:y2, x1:x2]
    
    # Convert to grayscale
    gray = cv2.cvtColor(chair_roi, cv2.COLOR_BGR2GRAY)
    
    # Create initial mask using thresholding
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    # Apply morphological operations to refine the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    
    return mask


def choose_point_on_edge(edges, x1, y1):
    global depth_frame
    
    # Find all edge points
    edge_points = np.argwhere(edges > 0)
    
    if edge_points.size == 0:
        return None
        
    # For each edge point, check depth and return closest valid point
    valid_points = []
    for point in edge_points:
        edge_y, edge_x = point
        depth = get_depth_at(edge_x + x1, edge_y + y1)
        
        if depth is not None and depth < 4.5:
            valid_points.append((edge_x + x1, edge_y + y1, depth))
            
    if not valid_points:
        return None
        
    # Return the point closest to the camera
    return min(valid_points, key=lambda x: x[2])


edge_x = 0
edge_y = 0
def tag_detections_callback(msg):
    if not msg.detections:
        rospy.loginfo("No AprilTags detected.")
        return

    for detection in msg.detections:
        tag_id = detection.id[0]
        pose = detection.pose.pose.pose  # geometry_msgs/Pose
        
        # Extract position (optional, for completeness)
        position = pose.position
        rospy.loginfo(f"Detected AprilTag ID: {tag_id}")
        rospy.loginfo(f"Position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})")

        # Extract orientation (quaternion)
        orientation = pose.orientation
        rospy.loginfo(f"Orientation (Quaternion): ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f})")
        
        # Optionally, convert the quaternion to Euler angles (pitch, yaw, roll)
        euler_angles = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        rospy.loginfo(f"Orientation (Euler angles): Roll = {euler_angles[0]:.2f}, Pitch = {euler_angles[1]:.2f}, Yaw = {euler_angles[2]:.2f}")

EDGE_COLOR = (0, 255, 0)  # Green color for edges
EDGE_WIDTH = 2           # Width of edge lines

def image_callback(msg):
    global frame, obj_dict, edge_x, edge_y
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

    focal_length_x = 924.2759399414062
    focal_length_y = 924.2759399414062

    center_x = 640.0
    center_y = 360.0

    intrinsic_matrix = np.array([[focal_length_x, 0, center_x],                         # Replace with actual values
                                 [0, focal_length_y, center_y],
                                 [0, 0, 1]])
    confidence_threshold = 0.8    
    results_sam = []
    chair_buff ={}
    buff_empty = True
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()                                           # Convert box tensor to list of coordinates
        bbox_center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
        
        conf = box.conf[0].item()
        cls = int(box.cls[0].item())
        class_name = model_det.names[cls]

        if conf < confidence_threshold:
            continue                                                                    
 
        # depth = get_depth_at(bbox_center[0], bbox_center[1])
        # X, Y, Z = pixel_to_3d(bbox_center[0], bbox_center[1], depth, intrinsic_matrix)  # Convert to 3D     
        
        edges = detect_edges(frame, int(x1), int(y1), int(x2), int(y2))
        point = choose_point_on_edge(edges, int(x1), int(y1))
        if point is not None:
            edge_x, edge_y, depth = point
        depth = get_depth_at(edge_x, edge_y)
        X, Y, Z = pixel_to_3d(edge_x, edge_y, depth, intrinsic_matrix)  # Convert to 3D     
        
        if class_name == "chair":
            results_sam = model_sam(frame, bboxes=[x1, y1, x2, y2])  # Append the result
            sam_mask = None
            for i, mask in enumerate(results_sam[0].masks.xy):
                if mask is not None:
                    sam_mask = np.array(mask, np.int32)
                    break                
                # depth = get_mask_depth_average(mask, depth_frame, num_samples=100)
                # X, Y, Z = pixel_to_3d(bbox_center[0], bbox_center[1], depth, intrinsic_matrix)  # Convert to 3D
            # Create mask for the chair region
            chair_mask = create_chair_mask(frame, int(x1), int(y1), int(x2), int(y2))
            
            # Detect edges using the mask
            edges = detect_edges(frame, int(x1), int(y1), int(x2), int(y2), chair_mask, sam_mask)
            
            # Choose a point on the edge with depth < 4.5 meters
            point = choose_point_on_edge(edges, int(x1), int(y1))
            # if point is not None:
            #     edge_x, edge_y, depth = point
            #     # pygame.draw.circle(screen, EDGE_COLOR, (int(edge_x), int(edge_y)), 5)
                
            #     # Draw edges as lines
            #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #     for contour in contours:
            #         if len(contour) >= 2:
            #             for i in range(len(contour)-1):
            #                 pt1 = (int(contour[i][0][0] + x1), int(contour[i][0][1] + y1))
            #                 pt2 = (int(contour[i+1][0][0] + x1), int(contour[i+1][0][1] + y1))
            #                 pygame.draw.line(screen, EDGE_COLOR, pt1, pt2, EDGE_WIDTH)
            
            # Draw bounding box
            pygame.draw.rect(screen, RED, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)), 5)
            
            
            if "chair" not in obj_dict:
                buff_empty = True
                chair_buff[f'chair_{cnt}'] = {"position": (0, 0, 0), "draw": (0, 0, 0, 0)}
                obj_dict["chair"] = {"position": (0.0, 0.0, 0.0), "detected": False}
            else:
                buff_empty = False
                chair_buff[f'chair_{cnt}'] = {"position":(X, Y, Z), "draw": (x1, y1, x2, y2)}
        cnt = cnt + 1
    print("chair_buff contents:", chair_buff)

    if buff_empty is False:
        max_x_chair = max(chair_buff.items(), key=lambda item: item[1]['position'][0])
        obj_dict["chair"]["position"] = max_x_chair[1]['position']
        X, Y, Z = max_x_chair[1]['position']
        x1, y1, x2, y2 = max_x_chair[1]['draw']
        obj_dict["chair"]["detected"] = True
        font = pygame.font.Font(None, 36)
        text_surface = font.render(f"chair: ({X:.2f}, {Y:.2f}, {Z:.2f})", True, (255, 255, 255))  
        screen.blit(text_surface, (x1, y1 - 40)) 
        pygame.draw.rect(screen, RED, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)), 5)                

        # Apply edge detection on the detected chair region
        edges = detect_edges(frame, int(x1), int(y1), int(x2), int(y2))
        # Choose a point on the edge with depth < 4.5 meters
        point = choose_point_on_edge(edges, int(x1), int(y1))
        if point is not None:
            edge_x, edge_y, depth = point
            # print("edge point is not none ...")
            pygame.draw.circle(screen, (0, 255, 0), (int(edge_x), int(edge_y)), 5)  # Draw the selected point in green  


    pygame.display.update()
        
    if boxes is None or len(boxes) == 0:

        obj_dict = {}
        # print('boxe is None')
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

def map_callback(msg):
    data = np.array(msg.data)
    total_cells = data.size
    known_cells = np.count_nonzero(data != -1)
    occupied_cells = np.count_nonzero(data == 100)
    free_cells = np.count_nonzero(data == 0)

    coverage_percent = 100.0 * known_cells / total_cells

    rospy.loginfo(f"Coverage: {coverage_percent:.2f}% | Known: {known_cells}/{total_cells} | Free: {free_cells} | Occupied: {occupied_cells}")

    
    
rgb_img_list = []
depth_img_list = []
def update_list(img_list, new_frame):
    """ Maintains only the last two frames in the list """
    img_list.append(new_frame)
    if len(img_list) > 2:
        img_list.pop(0)



bridge = CvBridge()
model_det = YOLO('yolo12n.pt') 
model_sam = SAM("sam_b.pt")

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
# rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_detections_callback)

sub_map = rospy.Subscriber("/projected_map", OccupancyGrid, map_callback)



while not rospy.is_shutdown():
    msg = json.dumps(obj_dict)
    pub_pos.publish(msg)


    rate.sleep()


