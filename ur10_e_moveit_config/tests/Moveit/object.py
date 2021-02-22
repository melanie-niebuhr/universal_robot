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

#Initialisiere MoveIt sowie Nodes und Publisher, instanziiere diverse Objekte
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur_node_me", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

## We can get the joint values from the group and adjust some of the values:

## Null_Position
joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

group.go(joint_goal, wait=True)
group.stop()

## Bewegung zu Joints
joint_goal = group.get_current_joint_values()
joint_goal[0] = -pi

group.go(joint_goal, wait=True)
group.stop()

## Null_Position
joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

group.go(joint_goal, wait=True)
group.stop()

## Adding Object
rospy.sleep(3)

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = 0.5
box_pose.pose.position.y = -0.5
box_pose.pose.position.z = 0.11
box_name = "box"
scene.add_box(box_name, box_pose, (0.2, 0.2, 0.2))

null_pose = group.get_current_pose().pose

# ## Bewegung zu Joints mit Object ohne Waypoints
# joint_goal = group.get_current_joint_values()
# joint_goal[0] = -pi
#
# group.go(joint_goal, wait=True)
# group.stop()
#
# ## Null_Position
# joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#
# group.go(joint_goal, wait=True)
# group.stop()

## Linearbewegung
pose = geometry_msgs.msg.Pose()
# pose.orientation.w = 0.5
pose.position.x = 1
pose.position.y = 0
pose.position.z = 0.3

group.set_pose_target(pose)

plan_1 = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()


## Neuplanung mit Waypoints
# You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through
scale = 1.0
waypoints = []

# First position
wpose = group.get_current_pose().pose
# wpose.orientation.x = 1
# wpose.orientation.y = 1
# wpose.orientation.z = 1
wpose.position.x = 0.8
wpose.position.y = -0.2 # 0.2
wpose.position.z = 0.05
waypoints.append(copy.deepcopy(wpose))

# Second position
# wpose.orientation.w = 0.5
wpose.position.z += scale * 0.45  # First move up (z)
waypoints.append(copy.deepcopy(wpose))

# Third position
# wpose.orientation.w = 0.5
wpose.position.x -= scale * 0.6  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

# Fourth position
# wpose.orientation.w = 0.5
wpose.position.z -= scale * 0.45  # move down (z)
wpose.position.y -= scale * 0.5  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

# Final position
# wpose.orientation.w = 0.5
# wpose.orientation.x = 1
# wpose.orientation.y = 1
# wpose.orientation.z = 1
wpose.position.x = -0.8
wpose.position.y = -0.5
wpose.position.z = 0.3
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan_1, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:

group.execute(plan_1, wait=True)

# We can also check if this group has a link that is considered to be an end effector:
eef_link = group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)
print("============ End effector link: %s" % waypoints)
print("============ Null_Pose: %s" % null_pose)


## Removing Object
rospy.sleep(5)
scene.remove_world_object(box_name)

group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

# rospy.sleep(5)
# group.set_pose_target(null_pose, end_effector_link = "eef_link" )
# group.go(wait=True)
# group.execute(plan_2, wait=True)

## Linearbewegung
fpose = geometry_msgs.msg.Pose()
fpose = null_pose
# fpose.position.x = null_pose.position.x
# fpose.position.y = null_pose.position.y
# fpose.position.z = null_pose.position.z

group.set_pose_target(fpose)

# plan_2 = group.go(wait=True)

group.go(wait=True)






