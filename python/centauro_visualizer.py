#!/usr/bin/python3
import rospy
import rospkg
import casadi as cs
import numpy as np
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from geometry_msgs.msg import PointStamped
import cartesian_interface.affine3


rospy.init_node('centauro_walk_visualizer')

print("ok to")


'''
Load urdf and srdf
'''
urdf = rospy.get_param(param_name='/robot_description', default='')
if urdf == '':
    raise print('urdf not set')
srdf = rospy.get_param(param_name='/robot_description_semantic', default='')
if srdf == '':
    raise print('srdf not set')

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

robot = xbot.RobotInterface(cfg)
model_fk = robot.model()
robot.sense()

data = np.zeros((3, 1))


contact1_pub = rospy.Publisher('contact1_pub', PointStamped, queue_size=10)
contact1_point = PointStamped()

contact2_pub = rospy.Publisher('contact2_pub', PointStamped, queue_size=10)
contact2_point = PointStamped()

contact3_pub = rospy.Publisher('contact3_pub', PointStamped, queue_size=10)
contact3_point = PointStamped()

contact4_pub = rospy.Publisher('contact4_pub', PointStamped, queue_size=10)
contact4_point = PointStamped()

rate = rospy.Rate(50)

while not rospy.is_shutdown():
    # print('rospy running')
    # =========================== publish contact position ========================
    pose = model_fk.getPose('contact_1')
    data[0] = pose.translation[0]
    data[1] = pose.translation[1]
    data[2] = pose.translation[2]
    contact1_pub = rospy.Publisher('contact1_pub', PointStamped, queue_size=10)
    contact1_point = PointStamped()
    contact1_point.header.stamp = rospy.Time.now()
    contact1_point.header.frame_id = 'world'
    contact1_point.point.x = data[0]
    contact1_point.point.y = data[1]
    contact1_point.point.z = 0
    contact1_pub.publish(contact1_point)

    pose = model_fk.getPose('contact_2')
    data[0] = pose.translation[0]
    data[1] = pose.translation[1]
    data[2] = pose.translation[2]
    contact2_pub = rospy.Publisher('contact2_pub', PointStamped, queue_size=10)
    contact2_point = PointStamped()
    contact2_point.header.stamp = rospy.Time.now()
    contact2_point.header.frame_id = 'world'
    contact2_point.point.x = data[0]
    contact2_point.point.y = data[1]
    contact2_point.point.z = 0
    contact2_pub.publish(contact2_point)

    pose = model_fk.getPose('contact_3')
    data[0] = pose.translation[0]
    data[1] = pose.translation[1]
    data[2] = pose.translation[2]
    contact3_pub = rospy.Publisher('contact3_pub', PointStamped, queue_size=10)
    contact3_point = PointStamped()
    contact3_point.header.stamp = rospy.Time.now()
    contact3_point.header.frame_id = 'world'
    contact3_point.point.x = data[0]
    contact3_point.point.y = data[1]
    contact3_point.point.z = 0
    contact3_pub.publish(contact3_point)

    pose = model_fk.getPose('contact_4')
    data[0] = pose.translation[0]
    data[1] = pose.translation[1]
    data[2] = pose.translation[2]
    contact4_pub = rospy.Publisher('contact4_pub', PointStamped, queue_size=10)
    contact4_point = PointStamped()
    contact4_point.header.stamp = rospy.Time.now()
    contact4_point.header.frame_id = 'world'
    contact4_point.point.x = data[0]
    contact4_point.point.y = data[1]
    contact4_point.point.z = 0
    contact4_pub.publish(contact4_point)
    # ============================================================================
    rate.sleep()