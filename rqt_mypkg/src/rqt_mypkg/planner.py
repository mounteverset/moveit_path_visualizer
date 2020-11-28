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

if __name__ == '__main__':

    planningObject = path_planning_interface.MoveGroupDefinedPath()
    planningObject.go_to_starting_pose()
    planningObject.go_to_goal_pose()

#def main (self):
    #to implement: error handling
    #planningObject = MoveGroupDefinedPath()
    #planningObject.go_to_starting_pose()
    #planningObject.go_to_goal_pose()
