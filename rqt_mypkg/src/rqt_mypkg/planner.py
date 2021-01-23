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

#joint_goal = planningObject.get_inverse_kinematic()
#Roboter begibt sich in die Ausgangstellung
#planningObject.go_to_starting_pose()
#plan = planningObject.plan_path_from_to_start_with_joint_goal()
planningObject.go_to_starting_joint_goal()
#es wird eine inverse Kinematik berechnet
#joint_goal = planningObject.get_inverse_kinematic()

#Roboter berechnet den Pfad zur Zielpose
planned_path = planningObject.plan_path_from_joint_goal()
if planned_path[0] == True:
    duration_from_start = planned_path[1].joint_trajectory.points[len(planned_path[1].joint_trajectory.points)-1].time_from_start
    execution_time = duration_from_start.secs + (duration_from_start.nsecs/1000000000)
    planning_time = planned_path[2]
    #Roboter holt sich alle Posen des EEF und speichert die Marker
    eef_poses = planningObject.get_eef_poses(planned_path)

    #Die Marker werden gepublished 
    marker_array = planningObject.create_eef_marker(eef_poses)

    #Für die Statistik wird die Pfadlänge berechnet
    path_length = statisticsObject.get_path_length(eef_poses)
    max_acceleration = statisticsObject.get_max_joint_acceleration(planned_path)
    planningObject.publish_statistics(path_length, marker_array, planning_time, max_acceleration, eef_poses, execution_time)
else:
    print("No motion plan found. Reason for shutdown: {}".format(planned_path[3]))
