#!/usr/bin/env python

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped

default_parent = "world"
default_child = "eef"

if len(sys.argv) < 2:
  print('Using default values for parent and child frame: "%s" and "%s"' % (default_parent, default_child))
  parent = default_parent
  child = default_child
elif len(sys.argv) == 2:
  print('Using default value for parent frame: %s"' % default_parent)
  parent = default_parent
  child = sys.argv[1]
elif len(sys.argv) == 3:
  parent = sys.argv[1]
  child = sys.argv[2]
else:
  print('Too many arguments. Usage:')
  print(' - find_pose.py')
  print('     uses default values: parent="%s", child="%s"' % (default_parent, default_child))
  print(' - find_pose.py child_frame')
  print('     uses default value: parent="%s"' % (default_parent))
  print(' - find_pose.py parent_frame child_frame')
  exit()

print "Setting up..."
rospy.init_node('find_pose', anonymous=True)

tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)
rospy.sleep(5.0)

transform = tf_buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(1.0))
print "====================="
print "Pose of %s with respect to %s:\n%s" % (parent, child, transform)