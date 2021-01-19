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

#Lösung des IK Problems -> Planung zuerst mit OMPL!
#Der Roboter fährt zu der Startposition die mit den Punkten vorgegeben wurden
#Der Roboter fährt zur Zielposition die mit den Pkt vorgegeben wurde
#Die Joint Values werden gespeichert in diesem Programm!
#Diese Joint Values können dann wieder benutzt werden, um CHOMP und STOMP zu benutzen

#Roboter soll zur Startpose fahren 
planningObject.go_to_starting_pose()

joint_goal = planningObject.get_inverse_kinematic()
#Roboter berechnet den Pfad zur Zielpose
planned_path = planningObject.plan_path_from_pose()
#print (planned_path[1])
#Roboter holt sich alle Posen des EEF und speichert die Marker
eef_poses = planningObject.get_eef_poses(planned_path)
#Roboter published die Marker
planningObject.display_eef_marker(eef_poses)

#planningObject.display_trajectory(planned_path[1])
#print (planningObject.move_group.get_current_pose().pose)
#planningObject.go_to_goal_pose()
#print (planningObject.move_group.get_current_pose().pose)
#rospy.signal_shutdown()
