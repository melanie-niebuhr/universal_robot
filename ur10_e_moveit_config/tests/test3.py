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



### Bewegung zu Joints
# joint_goal = group.get_current_joint_values()
# joint_goal[0] = pi
# joint_goal[1] = - pi / 2
# joint_goal[2] = - pi /2
# joint_goal[3] = - pi /3
# joint_goal[4] = pi / 2
# joint_goal[5] = 0
# group.go(joint_goal, wait=True)
# group.stop()


## Bewegung in Zielpose
pose_1 = group.get_current_pose().pose
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.5
pose_goal.position.y = 0.5
pose_goal.position.z = 0.5


#pose_goal.orientation.w = 0.5
pose_goal.orientation.x = 1.0
pose_goal.orientation.y = 1.0
pose_goal.orientation.z = 1.0


group.set_pose_target(pose_goal)
group.go(wait=True)
group.stop()
group.clear_pose_targets()

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "manipulator"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = 0.11 # above the panda_hand frame
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.075, 0.75, 0.75))

### Linearbewegung
pose_2 = group.get_current_pose().pose
pose = copy.deepcopy(pose_2)
pose.position.x=-0.7
pose.position.y=0.7
pose.position.z=0.6
waypoints = []
waypoints.append(copy.deepcopy(pose))
(plan, fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
group.execute(plan, wait=True)
