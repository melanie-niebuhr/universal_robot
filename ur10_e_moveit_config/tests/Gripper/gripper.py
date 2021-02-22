#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Initialisiere MoveIt sowie Nodes und Publisher, instanziiere diverse Objekte
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur_node_me", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

g_group = moveit_commander.MoveGroupCommander("gripper")
# open the gripper
g_group.set_named_target("opened")
plan2 = g_group.go()

## path planning
m_group = moveit_commander.MoveGroupCommander("manipulator")
# put the manipulator in the start position
m_group.set_named_target("home")
plan1 = m_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()