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



import colorama
ns = 10
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

pm = pymanager.PhaseManager(ns)
timeline_1 = pm.createTimeline('timeline_1')


# adding stuff to the problem
a = prb.createStateVariable('a', 1)
par = prb.createParameter('par', 1)
cnsrt_1 = prb.createConstraint('cnsrt_1', a / 2, nodes=[])

phase_1 = timeline_1.createPhase(4, 'phase_1')
# phase_1.addParameterValues(par, np.array([[1., 3., 4.]]), [0, 2, 3])

phase_2 = timeline_1.createPhase(6, 'phase_2')
# phase_2.addVariableBounds(a, np.array([[-1, -2, -3, -4, -5]]), np.array([[1, 2, 3, 4, 5]]))
# phase_2.addConstraint(cnsrt_1, nodes=[0, 3])

phase_inserted = timeline_1.createPhase(3, 'phase_inserted')

timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_2)


def printElemInfo(elem):
    print(f"  |     Phase position: {elem.getPosition()}")
    print(f"  |     Phase duration: {elem.getNNodes()}")
    print(f"  |     Active nodes of phase: {elem.getActiveNodes()}")
    print(f"  |_____________________________________________________________|")

def printAllPhases(timeline: pytimeline.Timeline, add_element_info=False):
    elem_num = 1
    print(f"{colorama.Style.BRIGHT}==================== ALL PHASES: ===================={colorama.Style.RESET_ALL}")
    for elem in timeline.getPhases():
        print(f"{colorama.Fore.RED}- {elem_num}: {colorama.Style.RESET_ALL}", end="")
        print(f"{elem.getName()}", end="")
        print(f": {elem}")
        if add_element_info:
            printElemInfo(elem)
        elem_num += 1

print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
printAllPhases(timeline_1, add_element_info=True)

for i in range(5):
    timeline_1.shift()
    print(i+1," SHIFTING PHASES: ")
    printAllPhases(timeline_1)

exit()