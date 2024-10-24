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







ns = 30
stance_duration = 15
flight_duration = 15
contacts ={'contact_1', 'contact_2', 'contact_3', 'contact_4'}
# phase manager 
pm = pymanager.PhaseManager(ns, True)
# Create timelines in phase manager: contact_1_timeline; contact_2_timeline; contact_3_timeline; contact_4_timeline; -> 40 nodes
c_timelines = dict()
for c in contacts:
    str_ = f'{c}_timeline'
    c_timelines[c] = pm.createTimeline(str_)



# Register phases in timelines: stance_phase, flight_phases
c_i = 0
for c in contacts: # c: contact_1, contact_2, contact_3, contact_4
    c_i += 1
    # stance phase normal
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}') # register stance_phase
    # flight phase
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}') # register flight_phase

# Add phasese in timelines
for c in contacts:
    stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    while c_timelines[c].getEmptyNodes() > 0:
        c_timelines[c].addPhase(stance)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~1~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    printAllPhases(c_timelines[c], add_element_info=True)


# gm = GaitManager(ti, pm, contact_phase_map)

# pm.shift()

# for c in contacts:
#     print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~2~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
#     printAllPhases(c_timelines[c], add_element_info=True)


# print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
# printAllPhases(timeline_1, add_element_info=True)

# for i in range(5):
#     timeline_1.shift()
#     print(i+1," SHIFTING PHASES: ")
#     printAllPhases(timeline_1)

exit()



