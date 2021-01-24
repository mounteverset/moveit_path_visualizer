#!/usr/bin/env python3

import sys
import copy
from moveit_commander import move_group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from math import pi, sqrt, pow
from std_msgs.msg import String
import io
import shutil
import json
#used to convert the points from the gui in a valid message for ros
from geometry_msgs.msg import Pose, PoseStamped
#used to read out the start points
import os
from nav_msgs.msg import Path
#used for publishing the planned path from start to goal
from visualization_msgs.msg import Marker, MarkerArray
#used to make a service request 
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from rqt_mypkg import path_planning_interface
from trajectory_msgs.msg import JointTrajectoryPoint

class StatisticsDefinedPath(object):


    def get_path_length(self, eef_poses):
        
        path_length = 0
        for i in range(len(eef_poses) - 1):
            posex = eef_poses[i].position.x
            posey = eef_poses[i].position.y
            posez = eef_poses[i].position.z

            posex1 = eef_poses[i+1].position.x
            posey1 = eef_poses[i+1].position.y
            posez1 = eef_poses[i+1].position.z

            path_length += sqrt(pow((posex1 - posex), 2) + pow((posey1- posey), 2))+ pow((posez1-posez),2)
        return path_length

    def get_max_joint_acceleration(self, motion_plan):
        maxlist = []

        for i in range(len(motion_plan[1].joint_trajectory.points)):
            for j in range(len(motion_plan[1].joint_trajectory.points[i].accelerations)):
                for k in range(len(motion_plan[1].joint_trajectory.points[i].accelerations)):
                    maxlist.append(motion_plan[1].joint_trajectory.points[i].accelerations[j])

        return max(maxlist)