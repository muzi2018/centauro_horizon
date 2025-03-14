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
model_sam = SAM("sam_b.pt")
        
def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        frame = frame.copy() 
        print("Converted image successfully")
    except Exception as e:
        print(f"Error converting image: {e}")
        return

    # Load a model
    cnt = 0
    # Run YOLO object detection on the frame
    results_det = model_det(frame, verbose=False)  # YOLO detection        
    detections = results_det[0]  # Get the first (and usually only) result from the list
        
    # Extract bounding boxes, class names, and confidence scores
    boxes = detections.boxes  # Bounding boxes in format (x1, y1, x2, y2)
    masks = detections.masks  # Bounding boxes in format (x1, y1, x2, y2)
    probs = detections.probs  # Confidence scores for each detection
    
    if boxes is None:
        print('boxes is None')
        # Handle lost detection for all tracked objects
        # handle_lost_detections()
        return


    focal_length_x = 924.2759399414062
    focal_length_y = 924.2759399414062

    center_x = 640.0
    center_y = 360.0

    intrinsic_matrix = np.array([[focal_length_x, 0, center_x],  # Replace with actual values
                                 [0, focal_length_y, center_y],
                                 [0, 0, 1]])
    results_sam = []
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()  # Convert box tensor to list of coordinates
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        depth = get_depth_at(cx, cy)  # Get depth value
        
        #get the 3d position in robot
        X, Y, Z = pixel_to_3d(cx, cy, depth, intrinsic_matrix)  # Convert to 3D
        
    
        
        conf = box.conf[0].item()
        cls = int(box.cls[0].item())
        class_name = model_det.names[cls]
        print(f"Detected {class_name} with confidence {conf:.2f} at [{x1}, {y1}, {x2}, {y2}]")
        print(f"Object {cnt} at ({cx}, {cy}) has depth: {depth} meters")
        print(f"3D position of object {cnt}: ({X:.2f}, {Y:.2f}, {Z:.2f}) meters")
        
        # print("Box:", box.xyxy.tolist())  # Check structure
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        # cv2.putText(frame, f"p ({X:.2f}, {Y:.2f}, {Z:.2f})", (int(cx), int(cy)),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.putText(frame, f"{class_name} ({conf:.2f})", (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        results_sam.append(model_sam(frame, bboxes=[x1, y1, x2, y2]))  # Append the result
        cnt = cnt + 1        
    print("\n\n")  
    
    mask_img = np.zeros_like(frame)  # Create an empty mask image
    for j in range(cnt):
        box = boxes[j]
        x1, y1, x2, y2 = box.xyxy[0].tolist()  # Convert box tensor to list of coordinates
        bbox_center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
        for i, mask in enumerate(results_sam[j][0].masks.xy):  # Loop through all detected masks
            mask = np.array(mask, np.int32)  # Convert mask to integer array
            cv2.fillPoly(mask_img, [mask], (0, 255, 0))  # Fill the segmented area with green
            # Compute the closest point on the mask to bbox_center
            mask_points = mask.reshape((-1, 2))
            distances = np.linalg.norm(mask_points - bbox_center, axis=1)
            closest_index = np.argmin(distances)
            closest_point = tuple(mask_points[closest_index])  # Nearest mask point
            center_x, center_y = closest_point
            print(f"Center of mask: ({center_x}, {center_y})")
            
            depth = get_depth_at(center_x, center_y)  # Get depth value
            #get the 3d position in robot
            X, Y, Z = pixel_to_3d(center_x, center_y, depth, intrinsic_matrix)  # Convert to 3D
            
            # Visualize the center on the frame
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"p ({X:.2f}, {Y:.2f}, {Z:.2f})", (int(center_x), int(center_y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    # Blend the segmentation mask with the original frame
    blended = cv2.addWeighted(frame, 0.7, mask_img, 0.3, 0)
    # Display the result
    cv2.imshow("Segmented Objects", blended)
    cv2.waitKey(1)


def handle_lost_detections():
    """Handle cases where objects are temporarily lost"""
    global tracker
    current_time = rospy.get_rostime()
    
    # Check each tracked object
    for obj_id in list(tracker.tracked_objects.keys()):
        obj = tracker.tracked_objects[obj_id]
        time_since_last_seen = (current_time - obj['last_seen']).to_sec()
        
        if time_since_last_seen > tracker.max_tracking_loss:
            # Object lost for too long, remove from tracking
            del tracker.tracked_objects[obj_id]
            print(f"Removed lost object: {obj_id}")
        else:
            # Predict position for temporarily lost object
            predicted_pos = tracker.predict_position(obj_id)
            if predicted_pos is not None:
                # Update visualization with predicted position
                visualize_predicted_position(predicted_pos, obj_id)

def visualize_predicted_position(position, obj_id):
    """Visualize predicted object position"""
    global frame
    if frame is None:
        return
        
    # Project 3D position back to 2D for visualization
    focal_length_x = 924.2759399414062
    focal_length_y = 924.2759399414062
    center_x = 640.0
    center_y = 360.0
    x = int(position[0] * focal_length_x / position[2] + center_x)
    y = int(position[1] * focal_length_y / position[2] + center_y)
    
    # Draw predicted position marker
    cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
    cv2.putText(frame, f"Pred {obj_id}", (x, y - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


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

rate = rospy.Rate(50)

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

while not rospy.is_shutdown():
    # print('perception')
    rate.sleep()
    # rospy.spin()



