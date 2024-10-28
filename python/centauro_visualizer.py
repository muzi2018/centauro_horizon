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
from centauro_horizon.msg import WBTrajectory

solution = {}
solution['q'] = np.zeros(23)
solution['v'] = np.zeros(22)
solution['a'] = np.zeros(22)
qdot = np.zeros(46) # 46 x 1
qddot = np.zeros(46) # 46 x 1
def solution_callback(msg):
    # solution['q'] = msg.q # 23
    # solution['v'] = msg.v # 22
    # solution['a'] = msg.a # 22
    qdot[7:11] = msg.v[0:4] # 7-10
    qdot[11:15] = msg.v[4:8] # 11-14
    qdot[15:19] = msg.v[8:12] # 15-18
    qdot[19:23] = msg.v[12:16] # 19-23

    qddot[7:11] = msg.a[0:4] # 7-10
    qddot[11:15] =msg.a[4:8] # 11-14
    qddot[15:19] =msg.a[8:12] # 15-18
    qddot[19:23] =msg.a[12:16] # 19-23
    print("q size: ", len(msg.q)) # 23
    print("v size: ", len(msg.v)) # 22
    print("a size: ", len(msg.a)) # 22   
rospy.init_node('centauro_walk_visualizer')

# print("ok to 1")


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

rate = rospy.Rate(20)
rospy.Subscriber('/mpc_solution', WBTrajectory, solution_callback, queue_size=1, tcp_nodelay=True)


ns = 40
T = 2.
dt = T / ns

while not rospy.is_shutdown():
    print('rospy running')
    q = model_fk.getJointPosition() # 46 # okay
    

    # qdot[7:11] = solution['v'][0:4].reshape(4, 1) # 7-10
    # qdot[11:15] = solution['v'][4:8].reshape(4, 1) # 11-14
    # qdot[15:19] = solution['v'][8:12].reshape(4, 1) # 15-18
    # qdot[19:23] = solution['v'][12:16].reshape(4, 1) # 19-23

    # qddot[7:11] = solution['a'][0:4].reshape(4, 1) # 7-10
    # qddot[11:15] = solution['a'][4:8].reshape(4, 1) # 11-14
    # qddot[15:19] = solution['a'][8:12].reshape(4, 1) # 15-18
    # qddot[19:23] = solution['a'][12:16].reshape(4, 1) # 19-23

    # q += dt * qdot.flatten() + 0.5 * pow(dt, 2) * qddot.flatten()
    # qdot += dt * qddot

    # model_fk.setJointPosition(q)
    # model_fk.setJointVelocity(qdot)
    # model_fk.setJointAcceleration(qddot)
    # model_fk.update()

    ## =========================== publish contact position ========================
    # pose = model_fk.getPose('contact_1')
    # data[0] = pose.translation[0]
    # data[1] = pose.translation[1]
    # data[2] = pose.translation[2]
    # print("contact_1 position", data[2])
    # contact1_pub = rospy.Publisher('contact1_pub', PointStamped, queue_size=10)
    # contact1_point = PointStamped()
    # contact1_point.header.stamp = rospy.Time.now()
    # contact1_point.header.frame_id = 'world'
    # contact1_point.point.x = data[0]
    # contact1_point.point.y = data[1]
    # contact1_point.point.z = data[2]
    # contact1_pub.publish(contact1_point)

    # pose = model_fk.getPose('contact_2')
    # data[0] = pose.translation[0]
    # data[1] = pose.translation[1]
    # data[2] = pose.translation[2]
    # contact2_pub = rospy.Publisher('contact2_pub', PointStamped, queue_size=10)
    # contact2_point = PointStamped()
    # contact2_point.header.stamp = rospy.Time.now()
    # contact2_point.header.frame_id = 'world'
    # contact2_point.point.x = data[0]
    # contact2_point.point.y = data[1]
    # contact2_point.point.z = data[2]
    # contact2_pub.publish(contact2_point)

    # pose = model_fk.getPose('contact_3')
    # data[0] = pose.translation[0]
    # data[1] = pose.translation[1]
    # data[2] = pose.translation[2]
    # contact3_pub = rospy.Publisher('contact3_pub', PointStamped, queue_size=10)
    # contact3_point = PointStamped()
    # contact3_point.header.stamp = rospy.Time.now()
    # contact3_point.header.frame_id = 'world'
    # contact3_point.point.x = data[0]
    # contact3_point.point.y = data[1]
    # contact3_point.point.z = data[2]
    # contact3_pub.publish(contact3_point)

    # pose = model_fk.getPose('contact_4')
    # data[0] = pose.translation[0]
    # data[1] = pose.translation[1]
    # data[2] = pose.translation[2]
    # contact4_pub = rospy.Publisher('contact4_pub', PointStamped, queue_size=10)
    # contact4_point = PointStamped()
    # contact4_point.header.stamp = rospy.Time.now()
    # contact4_point.header.frame_id = 'world'
    # contact4_point.point.x = data[0]
    # contact4_point.point.y = data[1]
    # contact4_point.point.z = data[2]
    # contact4_pub.publish(contact4_point)
    # ============================================================================
    rate.sleep()


    