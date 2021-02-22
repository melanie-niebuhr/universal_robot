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
## Endeffector Group
group_name_e = 'gripper'
group_e = moveit_commander.MoveGroupCommander(group_name_e)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
has_link = group_e.has_end_effector_link()
print("============ Has an End effector link: %s" % has_link)

# We can also check if this group has a link that is considered to be an end effector:
eef_link = group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

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

## Linearbewegung
pose = geometry_msgs.msg.Pose()
pose.orientation.w = 0.5
pose.position.x = 0.3
pose.position.y = 0.5
pose.position.z = 0.7

group.set_pose_target(pose)

plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

# You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through
scale = 1.0
waypoints = []

wpose = group.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:

group.execute(plan, wait=True)

# ## Random Box
rospy.sleep(2)
#
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = 0
box_pose.pose.position.y = 0
box_pose.pose.position.z = 0.11
box_name = "box"
scene.add_box(box_name, box_pose, (0.075, 0.075, 0.075))

## Box
# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = robot.get_planning_frame()
# box_pose.pose.orientation.w = 1.0
# box_pose.pose.position.z = 0.11 # above the panda_hand frame
# box_name = "box"
# scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

# start = rospy.get_time()
# seconds = rospy.get_time()
# while (seconds - start < timeout) and not rospy.is_shutdown():
#   # Test if the box is in attached objects
#   attached_objects = scene.get_attached_objects([box_name])
#   is_attached = len(attached_objects.keys()) > 0
#
#   # Test if the box is in the scene.
#   # Note that attaching the box will remove it from known_objects
#   is_known = box_name in scene.get_known_object_names()
#
#   # Test if we are in the expected state
#   if (box_is_attached == is_attached) and (box_is_known == is_known):
#     return True
#
#   # Sleep so that we give other threads time on the processor
#   rospy.sleep(0.1)
#   seconds = rospy.get_time()
#
# # If we exited the while loop without returning then we timed out
# return False

rospy.sleep(2)
grasping_group = 'gripper'
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)
#
# rospy.sleep(5)
# scene.remove_attached_object(eef_link, name=box_name)
#
# rospy.sleep(5)
# scene.remove_world_object(box_name)



