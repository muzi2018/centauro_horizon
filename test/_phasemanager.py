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


rospy.init_node('test_phasemanager')
urdf = rospy.get_param(param_name='/robot_description', default='')
if urdf == '':
    raise print('urdf not set')
srdf = rospy.get_param(param_name='/robot_description_semantic', default='')
if srdf == '':
    raise print('srdf not set')

'''
Initialize Horizon problem
'''
ns = 10
T = 2.
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

q_init = {
    # 'torso_yaw': 0.00,  # 0.00,
    # 'j_arm1_1': 1.50,  # 1.60,
    # 'j_arm1_2': 0.1,  # 0.,
    # 'j_arm1_3': 0.2,  # 1.5,
    # 'j_arm1_4': -2.2,  # 0.3,
    # 'j_arm1_5': 0.00,  # 0.00,
    # 'j_arm1_6': -1.3,  # 0.,
    # 'j_arm1_7': 0.0,    # 0.0,

    # 'j_arm2_1': 1.50,  # 1.60,
    # 'j_arm2_2': 0.1,  # 0.,
    # 'j_arm2_3': -0.2,  # 1.5,
    # 'j_arm2_4': -2.2,  # -0.3,
    # 'j_arm2_5': 0.0,  # 0.0,
    # 'j_arm2_6': -1.3,  # 0.,
    # 'j_arm2_7': 0.0,    # 0.0,

    # 'd435_head_joint': 0.0,
    # 'velodyne_joint': 0.0,
    'hip_yaw_1': -0.746,
    'hip_pitch_1': -1.254,
    'knee_pitch_1': -1.555,
    'ankle_pitch_1': -0.3,

    'hip_yaw_2': 0.746,
    'hip_pitch_2': 1.254,
    'knee_pitch_2': 1.555,
    'ankle_pitch_2': 0.3,

    'hip_yaw_3': 0.746,
    'hip_pitch_3': 1.254,
    'knee_pitch_3': 1.555,
    'ankle_pitch_3': 0.3,

    'hip_yaw_4': -0.746,
    'hip_pitch_4': -1.254,
    'knee_pitch_4': -1.555,
    'ankle_pitch_4': -0.3,
}
base_pose = np.array([0.07, 0., 0.8, 0., 0., 0., 1.])
base_twist = np.zeros(6)

wheels = [f'j_wheel_{i + 1}' for i in range(4)]
wheels_map = dict(zip(wheels, 4 * [0.]))

ankle_yaws = [f'ankle_yaw_{i + 1}' for i in range(4)]
ankle_yaws_map = dict(zip(ankle_yaws, [np.pi/4, -np.pi/4, -np.pi/4, np.pi/4]))

arm_joints = [f'j_arm1_{i + 1}' for i in range(6)] + [f'j_arm2_{i + 1}' for i in range(6)]
arm_joints_map = dict(zip(arm_joints, [0.75, 0.1, 0.2, -2.2, 0., -1.3, 0.75, 0.1, -0.2, -2.2, 0.0, -1.3]))

torso_map = {'torso_yaw': 0.}

head_map = {"dagana_2_claw_joint": 0.0,'d435_head_joint': 0.0, 'velodyne_joint': 0.0}

fixed_joint_map = dict()
fixed_joint_map.update(wheels_map)
fixed_joint_map.update(ankle_yaws_map)
fixed_joint_map.update(arm_joints_map)
fixed_joint_map.update(torso_map)

# replace continuous joints with revolute
urdf = urdf.replace('continuous', 'revolute')

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf, fixed_joints=fixed_joint_map)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_pose,
                                 fixed_joint_map=fixed_joint_map
                                 )
ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(rospkg.RosPack().get_path('centauro_horizon') + '/config/centauro_config.yaml')


pm = pymanager.PhaseManager(ns, True)

# phase manager handling
## contact_1_timeline; contact_2_timeline; contact_3_timeline; contact_4_timeline;
c_timelines = dict()
for c in model.cmap.keys():
    str_ = f'{c}_timeline'
    c_timelines[c] = pm.createTimeline(str_)


short_stance_duration = 5
stance_duration = 5
flight_duration = 15
c_i = 0


tg = trajectoryGenerator.TrajectoryGenerator()

for c in model.getContactMap(): # c: contact_1, contact_2, contact_3, contact_4
    c_i += 1
    # stance phase normal
    stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}') # register stance_phase
    # stance_phase.getTimelines()
    # stance_phase_short = c_timelines[c].createPhase(short_stance_duration, f'stance_{c}_short') # register stance_phase_short
    if ti.getTask(f'contact_{c_i}') is not None:
        stance_phase.addItem(ti.getTask(f'contact_{c_i}'))
        # stance_phase_short.addItem(ti.getTask(f'contact_{c_i}'))
    else:
        raise Exception('task not found')

    # flight phase
    flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}') # register flight_phase
    init_z_foot = model.kd.fk(c)(q=model.q0)['ee_pos'].elements()[2]
    ee_vel = model.kd.frameVelocity(c, model.kd_frame)(q=model.q, qdot=model.v)['ee_vel_linear']
    ref_trj = np.zeros(shape=[7, flight_duration])
    ref_trj[2, :] = np.atleast_2d(tg.from_derivatives(flight_duration, init_z_foot, init_z_foot + 0.01, 0.1, [None, 0, None]))
    if ti.getTask(f'z_contact_{c_i}') is not None:
        flight_phase.addItemReference(ti.getTask(f'z_contact_{c_i}'), ref_trj)
    else:
        raise Exception('task not found')

    cstr = prb.createConstraint(f'{c}_vert', ee_vel[0:2], [])
    flight_phase.addConstraint(cstr, nodes=[0, flight_duration-1])

    c_ori = model.kd.fk(c)(q=model.q)['ee_rot'][2, :]
    cost_ori = prb.createResidual(f'{c}_ori', 5. * (c_ori.T - np.array([0, 0, 1])))
    flight_phase.addCost(cost_ori)

for c in model.cmap.keys():
    stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
    while c_timelines[c].getEmptyNodes() > 0:
        # print("EmptyNodes: ", c_timelines[c].getEmptyNodes())
        c_timelines[c].addPhase(stance)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~",c,"_timeline~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    printAllPhases(c_timelines[c], add_element_info=True)

contact_phase_map = {c: f'{c}_timeline' for c in model.cmap.keys()}
gm = GaitManager(ti, pm, contact_phase_map)
gait_manager_ros = GaitManagerROS(gm)

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)
f0 = [0, 0, kin_dyn.mass() / 4 * 9.8]
for cname, cforces in ti.model.cmap.items():
    for c in cforces:
        c.setInitialGuess(f0)
vel_lims = model.kd.velocityLimits()
prb.createResidual('max_vel', 1e1 * utils.barrier(vel_lims[7:] - model.v[7:]))
prb.createResidual('min_vel', 1e1 * utils.barrier1(-1 * vel_lims[7:] - model.v[7:]))
ti.finalize()
ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution
rate = rospy.Rate(1 / dt)
wrench_pub = rospy.Publisher('centauro_base_estimation/contacts/set_wrench', ContactWrenches, latch=False, queue_size =1)

while not rospy.is_shutdown():

    pm.shift()
    timeline_1 = pm.getTimelines('contact_1_timeline')
    # print("---------------SHIFT-----------------")
    # printAllPhases(timeline_1, add_element_info=True)


    gait_manager_ros.run()
    rate.sleep()


# ns = 30
# stance_duration = 15
# flight_duration = 15
# contacts ={'contact_1', 'contact_2', 'contact_3', 'contact_4'}
# # phase manager 
# pm = pymanager.PhaseManager(ns, True)
# # Create timelines in phase manager: contact_1_timeline; contact_2_timeline; contact_3_timeline; contact_4_timeline; -> 40 nodes
# c_timelines = dict()
# for c in contacts:
#     str_ = f'{c}_timeline'
#     c_timelines[c] = pm.createTimeline(str_)



# # Register phases in timelines: stance_phase, flight_phases
# c_i = 0
# for c in contacts: # c: contact_1, contact_2, contact_3, contact_4
#     c_i += 1
#     # stance phase normal
#     stance_phase = c_timelines[c].createPhase(stance_duration, f'stance_{c}') # register stance_phase
#     # flight phase
#     flight_phase = c_timelines[c].createPhase(flight_duration, f'flight_{c}') # register flight_phase

# # Add phasese in timelines
# for c in contacts:
#     stance = c_timelines[c].getRegisteredPhase(f'stance_{c}')
#     while c_timelines[c].getEmptyNodes() > 0:
#         c_timelines[c].addPhase(stance)
#     print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~1~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
#     printAllPhases(c_timelines[c], add_element_info=True)


# # gm = GaitManager(ti, pm, contact_phase_map)

# # pm.shift()

# # for c in contacts:
# #     print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~2~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
# #     printAllPhases(c_timelines[c], add_element_info=True)


# # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
# # printAllPhases(timeline_1, add_element_info=True)

# # for i in range(5):
# #     timeline_1.shift()
# #     print(i+1," SHIFTING PHASES: ")
# #     printAllPhases(timeline_1)

exit()



