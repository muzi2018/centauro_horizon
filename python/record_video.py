import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_matrix

directory = "/home/wang/room"
os.makedirs(directory, exist_ok=True)

bridge = CvBridge()
image_count = 0
depth_count = 0

camera_pose_file = os.path.join(directory, "traj.txt")
camera_img_file = os.path.join(directory, "results")
with open(camera_pose_file, "w") as f:
    pass  # Create an empty file or clear existing content

def save_camera_pose(pose):
    rate = rospy.Rate(3)  # Limit the rate of this callback to 3 Hz
    try:
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        print("camera position: ", position)
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = position
        
        flattened_matrix = transformation_matrix.flatten()
        
        with open(camera_pose_file, "a") as f:
            np.savetxt(f, [flattened_matrix], fmt='%f')
    except Exception as e:
        rospy.logerr(f"Error writing to camera_poses.txt: {e}")
    rate.sleep()  # Sleep to control callback rate



def image_callback(msg):
    global image_count
    rate = rospy.Rate(3)  # Limit the rate of this callback to 3 Hz

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        filename = os.path.join(camera_img_file, f"frame_{image_count:06d}.jpg")
        cv2.imwrite(filename, cv_image)
        rospy.loginfo(f"Saved {filename}")
        image_count += 1
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")
    rate.sleep()  # Sleep to control callback rate

def depth_callback(msg):
    global depth_count
    rate = rospy.Rate(3)  # Limit the rate of this callback to 3 Hz

    try:
        cv_depth = bridge.imgmsg_to_cv2(msg, "16UC1")
        filename = os.path.join(camera_img_file, f"depth_{depth_count:06d}.png")
        cv2.imwrite(filename, cv_depth)
        rospy.loginfo(f"Saved {filename}")
        depth_count += 1
    except Exception as e:
        rospy.logerr(f"Error processing depth image: {e}")
    rate.sleep()  # Sleep to control callback rate

def gazebo_callback(msg):
    try:
        camera_name = "centauro::d435_head_motor"
        if camera_name in msg.name:
            index = msg.name.index(camera_name)
            save_camera_pose(msg.pose[index])
    except Exception as e:
        rospy.logerr(f"Error retrieving camera pose: {e}")


def main():
    rospy.init_node("image_recorder", anonymous=True)
    rospy.Subscriber("/D435_head_camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/D435_head_camera/aligned_depth_to_color/image_raw", Image, depth_callback)
    rospy.Subscriber("/gazebo/link_states", ModelStates, gazebo_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
