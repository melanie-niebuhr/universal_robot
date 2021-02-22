#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



#Initialisiere MoveIt sowie Nodes und Publisher, instanziiere diverse Objekte
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur_node_me", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
group_g = moveit_commander.MoveGroupCommander("gripper")

## Current values of joints
print "============ Printing Current Joint Value:"
print group.get_current_joint_values()

## Current Pose position
print "============ Printing Current Pose Position:"
print group.get_current_pose()

## Endeffector Group
# We can also print the name of the end-effector link for this group:
has_link = group.has_end_effector_link()
print("============ Has an End effector link: %s" % has_link)


# ## Current values of joints
print "============ Printing Current Joint Value:"
print group_g.get_current_joint_values()
#
# ## Current Pose position --> No Endeffektor to get the pose of!!
# print "============ Printing Current Pose Position:"
# print group_g.get_current_pose()

## Endeffector Group
# We can also print the name of the end-effector link for this group:
has_link = group_g.has_end_effector_link()
print("============ Has an End effector link: %s" % has_link)