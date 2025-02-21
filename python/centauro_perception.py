#!/usr/bin/python3
import rospy
import rospkg
import casadi as cs
import numpy as np
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import subprocess
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline
import phase_manager.pyrosserver as pyrosserver
import horizon.utils.analyzer as analyzer

from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.problem import Problem
from horizon.utils import trajectoryGenerator, resampler_trajectory, utils, analyzer
from centauro_horizon.msg import WBTrajectory
from std_msgs.msg import Float64

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from horizon.rhc.taskInterface import TaskInterface
from horizon.rhc.gait_manager import GaitManager
from horizon.rhc.ros.gait_manager_ros import GaitManagerROS
from geometry_msgs.msg import Wrench
from scipy.spatial.transform import Rotation
from base_estimation.msg import ContactWrenches
import cartesian_interface.roscpp_utils as roscpp
import cartesian_interface.pyci as pyci
import cartesian_interface.affine3
import time

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import colorama
# exit()
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

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    print('perception')
    rate.sleep()


