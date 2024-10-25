#!/usr/bin/python3
import rospy
import rospkg
from xbot_interface import xbot_interface as xbot
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
xbot
'''
xbot_param = rospy.get_param(param_name="~xbot", default=False)
if xbot_param:
    print ("xbot work")
    # exit()
    robot = xbot.RobotInterface(cfg)
    model_fk = robot.model()


    q_init = robot.getJointPosition()
    q_init = robot.eigenToMap(q_init)
else:
    print("xbot not work")