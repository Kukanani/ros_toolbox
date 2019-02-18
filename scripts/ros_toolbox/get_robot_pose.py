#!/usr/bin/env python

import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

current_pose = group.get_current_pose().pose
print(current_pose)
print(group.get_planning_frame())
print(group.get_pose_reference_frame())
group.set_pose_reference_frame("j2s7s300_link_base")
current_pose = group.get_current_pose().pose
print(current_pose)
print(group.get_planning_frame())
print(group.get_pose_reference_frame())