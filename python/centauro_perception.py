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

# exit()




bridge = CvBridge()
depth_image = None  # Global variable to store depth data
def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        print("Converted depth image successfully")
    except Exception as e:
        print(f"Error converting depth image: {e}")
        
def image_callback(msg):
    global depth_image
    if depth_image is None:
        print("No depth image available yet")
        return

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color range for object detection (red in this case)
    lower_bound = np.array([0, 120, 70])
    upper_bound = np.array([10, 255, 255])

    # Mask and detect object
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w // 2, y + h // 2  # Center of detected object

        # Get depth value at (cx, cy)
        if depth_image is not None and 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
            depth_value = depth_image[cy, cx]
        else:
            depth_value = None

        if depth_value is not None and depth_value > 0:
            print(f"Object detected at: (X: {cx}, Y: {cy}, Depth: {depth_value}m)")

        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f"Depth: {depth_value:.2f}m" if depth_value else "Depth: N/A",
                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Ensure OpenCV displays the image
    cv2.namedWindow("Detected Object", cv2.WINDOW_NORMAL)
    cv2.imshow("Detected Object", frame)
    cv2.waitKey(1)  # Must be called to refresh the window


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

rate = rospy.Rate(10)

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

# rospy.Subscriber("/D435_head_camera/color/image_raw", Image, image_callback) 
# rospy.Subscriber("/D435_head_camera/aligned_depth_to_color/image_raw", Image, image_callback) 
# rospy.Subscriber("/D435_head_camera/depth/color/points", Image, image_callback) 


rospy.Subscriber("/D435_head_camera/aligned_depth_to_color/image_raw", Image, depth_callback)

while not rospy.is_shutdown():
    # print('perception')
    rate.sleep()
    # rospy.spin()



