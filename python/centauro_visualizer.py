#!/usr/bin/python3
import rospy
import rospkg
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
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

while not rospy.is_shutdown():
    print('rospy running')

