#!/usr/bin/python3
import rospy
import rospkg
import casadi as cs
import numpy as np
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3

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
while not rospy.is_shutdown():
    print('rospy running')
    # =========================== publish contact position ========================
    pose = model_fk.getPose('contact_1')
    data[0] = pose.translation[0]
    data[1] = pose.translation[1]
    data[2] = pose.translation[2]
    print("data: ", data)
    # ============================================================================
