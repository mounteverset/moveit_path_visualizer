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
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose

# This script is called by the rqt plugin and plans/executes a motion plan for a 
# given set of points by the plugin user. The script plans the robot motions and 
# collects data from the robot trajectory. The data and the plan get published 
# to a topic so that the rqt plugin can subscribe to it and receive the metrics 
# after invoking this script. This script uses the functions provided by the 
# path_planning_interface.py which interfaces the functionality of the MoveIt API.

planningObject = path_planning_interface.MoveGroupDefinedPath()

# If no inverse kinematic could be found for the start or goal pose the program 
# aborts and publishes an empty message for the plugin to pick up and invoke further action from the user
if len(planningObject.starting_joint_goal.position) == 0 or len(planningObject.goal_joint_goal.position) == 0:
    planningObject.publish_statistics(0, MarkerArray(), 0, 0, [Pose(),Pose()], 0)

else:
    statisticsObject = statistics.StatisticsDefinedPath()

    planningObject.go_to_starting_joint_goal()

    planned_path = planningObject.plan_path_from_joint_goal()

    # If a motion plan could be calculated the motion plan will be executed, the statistics will be calculated 
    # and published together with the motion plan. Otherwise a custom message will be published for the rqt 
    # plugin to pick up and invoke further actions from the user.
    if planned_path[0] == True:
        duration_from_start = planned_path[1].joint_trajectory.points[len(planned_path[1].joint_trajectory.points)-1].time_from_start
        execution_time = duration_from_start.secs + (duration_from_start.nsecs/1000000000)
        planning_time = planned_path[2]
        
        eef_poses = planningObject.get_eef_poses(planned_path)

        marker_array = planningObject.create_eef_marker(eef_poses)

        path_length = statisticsObject.get_path_length(eef_poses)
        max_acceleration = statisticsObject.get_max_joint_acceleration(planned_path)
        planningObject.publish_statistics(path_length, marker_array, planning_time, max_acceleration, eef_poses, execution_time)
    else:
        planningObject.publish_statistics(1, MarkerArray(), 1, 1, [Pose(),Pose()], 1)   
        print("No motion plan found. Reason for shutdown: {}".format(planned_path[3]))
