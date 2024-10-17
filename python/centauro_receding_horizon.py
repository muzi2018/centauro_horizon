#!/usr/bin/python3
import rospy
import rospkg

import cartesian_interface.roscpp_utils as roscpp

from centauro_horizon.msg import WBTrajectory
from std_msgs.msg import Float64


rospy.init_node('centauro_walk_srbd')
roscpp.init('centauro_walk_srbd', [])

solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=1, tcp_nodelay=True)
solution_time_publisher = rospy.Publisher('/mpc_solution_time', Float64, queue_size=1, tcp_nodelay=True)
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

'''
Initialize Horizon problem
'''
ns = 40
T = 2.
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

rate = rospy.Rate( 100 )
while not rospy.is_shutdown():
    print("Hello world!")
    rate.sleep()


