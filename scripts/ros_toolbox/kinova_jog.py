from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

import numpy as np
import rospy

import math

from sensor_msgs.msg import JointState
from kinova_msgs.msg import JointVelocity

robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, "j2s7s300_link_base", "j2s7s300_ee_link")

# q = kdl_kin.random_joint_angles()

q = None
eff = None
def cb_joint_states(joint_states):
  global q, eff
  q = np.array(joint_states.position[:7])
  eff = np.array(list(joint_states.effort[:7]))

rospy.init_node("test")
s = rospy.Subscriber("/j2s7s300_driver/out/joint_state", JointState, cb_joint_states)
jv_pub = rospy.Publisher("/j2s7s300_driver/in/joint_velocity", JointVelocity, queue_size=1)

while q is None and eff is None:
  rospy.sleep(0.001)

J = np.array(kdl_kin.jacobian(q))
bias = J.dot(eff)
start_position = np.array(kdl_kin.forward(q))
print(start_position)
while not rospy.is_shutdown():
  # get pseudoinverse jacobian
  J = np.array(kdl_kin.jacobian(q))
  J_inv = np.linalg.pinv(J)

  # delta_eef = J.dot(eff) - bias
  # dist_from_start = np.array(kdl_kin.forward(q)).dot(np.linalg.inv(start_position))

  # direction to move
  # x, y, z, rx, ry, rz
  # base coordinates, but located at the end effector
  # i.e. rz will rotate the robot's eef around the global z axis, but in
  # place rather than rotating around the base of the arm.
  # Not sure if that makes much sense but there you have it.
  # v = [0, 0, 1, 0.0, 0, 0.0]

  # move in a circle
  eef_pos = kdl_kin.forward(q)[:3, -1]
  target = [0, -0.3, 0.5, math.pi / 2, math.pi / 2, math.pi / 2]
  speed = 1
  v = (target - eef_pos) * speed
  # v = -delta_eef
  # v += -dist_from_start[:3, -1]
  jv = J_inv.dot(v)

  # jv = -delta_eef

  jv_msg = JointVelocity()
  jv_msg.joint1, jv_msg.joint2, jv_msg.joint3, \
    jv_msg.joint4, jv_msg.joint5, jv_msg.joint6, \
    jv_msg.joint7 = jv.tolist()

  jv_pub.publish(jv_msg)
  rospy.sleep(0.01)
