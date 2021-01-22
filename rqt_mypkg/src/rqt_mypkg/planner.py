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
from rqt_mypkg import statistics
from rqt_mypkg.msg import PathStatistics


planningObject = path_planning_interface.MoveGroupDefinedPath()
statisticsObject = statistics.StatisticsDefinedPath()

    #Roboter begibt sich in die Ausgangstellung
planningObject.go_to_starting_pose()

    #es wird eine inverse Kinematik berechnet
joint_goal = planningObject.get_inverse_kinematic()

    #Roboter berechnet den Pfad zur Zielpose
planned_path = planningObject.plan_path_from_pose()

planning_time = planned_path[2]
    #Roboter holt sich alle Posen des EEF und speichert die Marker
eef_poses = planningObject.get_eef_poses(planned_path)

    #Die Marker werden gepublished 
marker_array = planningObject.display_eef_marker(eef_poses)

    #Für die Statistik wird die Pfadlänge berechnet
path_length = statisticsObject.get_path_length(eef_poses)
max_acceleration = statisticsObject.get_max_joint_acceleration(planned_path)
planningObject.publish_statistics(path_length, marker_array, planning_time, max_acceleration, eef_poses)
    #planningObject.display_trajectory(planned_path[1])
    #print (planningObject.move_group.get_current_pose().pose)
    #planningObject.go_to_goal_pose()
    #print (planningObject.move_group.get_current_pose().pose)
    #rospy.signal_shutdown()
