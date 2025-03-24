#!/usr/bin/env python

import rospy
from sensor_msgs import point_cloud2
import pcl
from sensor_msgs.msg import PointCloud2

def publish_pcd():
    rospy.init_node('pcd_publisher', anonymous=True)
    pub = rospy.Publisher('/octomap_velodyne', PointCloud2, queue_size=1)
    
    # Load PCD file
    cloud = pcl.load('output.pcd')

    # Convert to ROS PointCloud2 message
    # Create header
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # Set appropriate frame

    # Get points from the PCL cloud
    points = cloud.to_array()

    # Convert to ROS PointCloud2 message
    ros_cloud = point_cloud2.create_cloud_xyz32(header, points)

    while not rospy.is_shutdown():
        pub.publish(ros_cloud)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        publish_pcd()
    except rospy.ROSInterruptException:
        pass
