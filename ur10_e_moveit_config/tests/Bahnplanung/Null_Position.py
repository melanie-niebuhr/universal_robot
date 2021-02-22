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
rospy.init_node("ur_node_ge", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

# Current values of joints
# print "============ Printing Current Joint Value:"
# print group.get_current_joint_values()

# joint_goal = group.get_current_joint_values()

joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# joint_goal[0] = 0
# joint_goal[1] = 0
# joint_goal[2] = 0
# joint_goal[3] = 0
# joint_goal[4] = 0
# joint_goal[5] = 0

group.go(joint_goal, wait=True)
group.stop()


planning_frame = robot.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

rospy.sleep(2)
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_name = "box"
scene.add_box(box_name, box_pose, (0.75, 0.75, 0.75))


rospy.sleep(2)
eef_link = group.get_end_effector_link()
grasping_group = 'gripper'
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)

rospy.sleep(2)
scene.remove_attached_object(eef_link, name=box_name)

rospy.sleep(2)
scene.remove_world_object(box_name)

## Current values of joints
print "============ Printing Current Joint Value:"
print group.get_current_joint_values()

## Current Pose position
print "============ Printing Current Pose Position:"
print group.get_current_pose()

