#!/usr/bin/python3
import rospy

rospy.init_node('centauro_walk_srbd')
rate = rospy.Rate( 100 )
while not rospy.is_shutdown():
    print("Hello world!")
    rate.sleep()


