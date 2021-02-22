#!/usr/bin/env python

# Python 2/3 compatibility imports
# from __future__ import print_function
# from six.moves import input

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

# We can also check if this group has a link that is considered to be an end effector:
eef_link = group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

## We can get the joint values from the group and adjust some of the values:
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)
# Calling ``stop()`` ensures that there is no residual movement
group.stop()

## Random Box
rospy.sleep(2)

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.orientation.w = 1
box_pose.pose.position.x = 0
box_pose.pose.position.y = 0
box_pose.pose.position.z = 0.2
box_name = "box"
scene.add_box(box_name, box_pose, (0.3, 0.1, 0.075))

rospy.sleep(2)

grasping_group = 'manipulator'
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)

## Attach object
# grasping_group = 'endeffector'
# touch_links = robot.get_link_names(group=grasping_group)
# group.attach_object(eef_link, box_name, touch_links=touch_links)

## Get a list of 3 elements defining the [roll, pitch, yaw] of the end-effector
rpy = group.get_current_rpy()
print("============ roll, pitch, yaw: %s" % rpy)

## Get the tolerance for achieving a joint goal (distance for each joint variable)
joint_tol = group.get_goal_joint_tolerance()
print("============ Joint Tolerance: %s" % joint_tol)

## When moving to an orientation goal or to a pose goal, the tolerance for the goal
## orientation is specified as the distance (roll, pitch, yaw) to the target origin of the end-effector
orient_tol = group.get_goal_orientation_tolerance()
print("============ Goal Orientation Tolerance: %s" % orient_tol)

## Joint Value Target
joint_target = group.get_joint_value_target()
print("============ Joint Value Target: %s" % joint_target)

## Joint Value Target
joints = group.get_joints()
print("============ Joints: %s" % joints)

## Get a list of names for the constraints specific for this group, as read from the warehouse
constraints = group.get_known_constraints()
print("============ Known Constraints: %s" % constraints)

# ## Random Joint Values
# random_joint = group.get_random_joint_values()
# print("============ Random Joint Values: %s" % random_joint)
#
# ## Random Pose
# random_pose = group.get_random_pose()
# print("============ Random Pose: %s" % random_pose)

## Return the number of variables used to parameterize a state in this group (larger or equal to number of DOF)
var_amount = group.get_variable_count()
print("============ Number of Variables: %s" % var_amount)

## Pick the named object. A grasp message, or a list of Grasp messages can also be specified as argument.
# group.pick('box', grasp=[])

## Specify a target orientation for the end-effector. Any position of the end-effector is acceptable.
rpy_orientation = [1, 0.0, 1]
group.set_rpy_target(rpy_orientation, end_effector_link = "eef_link")


