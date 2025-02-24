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


bridge = CvBridge()

# Load YOLO model
model = YOLO('yolo11n.pt')  # You can change the model to another pre-trained model (e.g., yolov8s.pt)

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
        print("get the depth_frame")
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
        
        
        
def image_callback(msg):
    print("Converted image successfully")
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        frame = frame.copy() 
        print("Converted image successfully")
    except Exception as e:
        print(f"Error converting image: {e}")
        return

    cnt = 1
    # Run YOLO object detection on the frame
    results = model(frame, verbose=False)  # YOLO detection        
    detections = results[0]  # Get the first (and usually only) result from the list
        
    # Extract bounding boxes, class names, and confidence scores
    boxes = detections.boxes  # Bounding boxes in format (x1, y1, x2, y2)
    # # names = detections.names  # Dictionary mapping class IDs to class names
    probs = detections.probs  # Confidence scores for each detection
    
    if boxes is None:
        print('boxes is None')
        
    # if probs is None:
    #     print('probs is None')

    focal_length_x = 924.2759399414062
    focal_length_y = 924.2759399414062

    center_x = 640.0
    center_y = 360.0

    intrinsic_matrix = np.array([[focal_length_x, 0, center_x],  # Replace with actual values
                                 [0, focal_length_y, center_y],
                                 [0, 0, 1]])

    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()  # Convert box tensor to list of coordinates
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        depth = get_depth_at(cx, cy)  # Get depth value
        
        #get the 3d position in robot
        X, Y, Z = pixel_to_3d(cx, cy, depth, intrinsic_matrix)  # Convert to 3D
        
    
        
        conf = box.conf[0].item()
        cls = int(box.cls[0].item())
        class_name = model.names[cls]
        print(f"Detected {class_name} with confidence {conf:.2f} at [{x1}, {y1}, {x2}, {y2}]")
        print(f"Object {cnt} at ({cx}, {cy}) has depth: {depth} meters")
        print(f"3D position of object {cnt}: ({X:.2f}, {Y:.2f}, {Z:.2f}) meters")

        # print("Box:", box.xyxy.tolist())  # Check structure
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"p ({X}, {Y}, {Z})", (int(cx), int(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.putText(frame, f"{class_name} ({conf:.2f})", (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        cnt = cnt + 1
        
    print("\n\n")  # Prints two empty lines
    # Ensure OpenCV displays the image
    cv2.namedWindow("YOLO Object Detection", cv2.WINDOW_NORMAL)
    cv2.imshow("YOLO Object Detection", frame)
    cv2.waitKey(1)  # Must be called to refresh the window






rospy.init_node('centauro_camera')
rospy.sleep(1.)    

rate = rospy.Rate(10)


rospy.Subscriber("/D435i_camera/color/image_raw", Image, image_callback) 
# rospy.Subscriber("/D435_camera/aligned_depth_to_color/image_raw", Image, depth_callback) 

while not rospy.is_shutdown():
    print('perception2')

    rate.sleep()

rospy.spin()

