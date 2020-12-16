#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from rqt_mypkg import path_planning_interface

#rospy.init_node("planner")

#while not rospy.is_shutdown():
planningObject = path_planning_interface.MoveGroupDefinedPath()

planned_path = planningObject.plan_path()

print (planned_path[1])

#planningObject.display_trajectory(planned_path[1])
eef_poses = planningObject.get_eef_poses(planned_path)
#planningObject.go_to_starting_pose()
planningObject.display_eef_marker(eef_poses)
print (planningObject.move_group.get_current_pose().pose)
#planningObject.go_to_goal_pose()
#print (planningObject.move_group.get_current_pose().pose)
#rospy.signal_shutdown()
