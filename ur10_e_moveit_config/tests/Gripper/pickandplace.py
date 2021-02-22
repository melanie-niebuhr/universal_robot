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
m_group = moveit_commander.MoveGroupCommander("manipulator")
g_group = moveit_commander.MoveGroupCommander("gripper")

## path planning
# put the manipulator in the start position
m_group.set_named_target("elbow")
plan1 = m_group.go()

# open the gripper
g_group.set_named_target("opened")
plan2 = g_group.go()

rospy.sleep(5)

## box
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = 0.48
box_pose.pose.position.y = 0.29
box_pose.pose.position.z = 0.115
box_name = "box"
scene.add_box(box_name, box_pose, (0.072, 0.072, 0.072))

# # Test if the box is in attached objects
# attached_objects = scene.get_attached_objects([box_name])
# is_attached = len(attached_objects.keys()) > 0
# print("============ Is attached: %s" % is_attached)
#
# rospy.sleep(2)
# grasping_group = 'gripper'
# touch_links = robot.get_link_names(group=grasping_group)
# scene.attach_box(link_name , box_name, touch_links=touch_links) #-- No Endeffector!
rospy.sleep(5)

# g_group.pick(box_name, grasp = [])
#
# g_group.place(box_name, location = None)

# semi-close the gripper
g_group.set_named_target("semi-closed")
plan3 = g_group.go()

# put the manipulator in the start position
m_group.set_named_target("home")
plan4 = m_group.go()

rospy.sleep(2)
scene.remove_world_object(box_name)

rospy.sleep(5)
moveit_commander.roscpp_shutdown()