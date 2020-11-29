#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import io

class MoveGroupDefinedPath(object):

    def __init__(self):
        super(MoveGroupDefinedPath, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_defined_path', anonymous=True)
        print("rospy Node started..")
        self.box_name = ""
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
    
    def go_to_starting_pose(self):
        
        print("Current joint_goals:")
        print (self.robot.get_current_state())
        start_pose = geometry_msgs.msg.Pose()
       
        pose = self.get_values(False)
        print(pose)
        start_pose.position.x = 0.8
        start_pose.position.y = 0.6
        start_pose.position.z = 0.1
        start_pose.orientation.w = 0.5
        print(start_pose)
        #setting the new joint goals
        # better would to set them as inverse kinematic
        print("Press Enter to go to starting pose")
        self.move_group.go(start_pose)
        print("noch in starting pose")
        self.move_group.stop()
    
    def go_to_goal_pose(self):
        print("jetzt in goal pose")
        pose_goal = geometry_msgs.msg.Pose()

        #get_values() from the saved file
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        pose_goal.orientation.w = 1.0
        print(pose_goal)

        self.move_group.set_pose_target(pose_goal)

        print("Press Enter to plan the path and go to pose goal")
        plan = self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()

        pausing = input()

    def get_values(self, goal):
        if (goal == True):
            with open("goal_pose.txt", "r") as f:
                pose = [line.rstrip() for line in f]
        elif (goal != True):
            with open("starting_pose.txt", "r") as f:
                pose = [line.rstrip() for line in f]
        else:
            return
        return pose     