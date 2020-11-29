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
        
        #pause zum durchlesen der infos im terminal
        pose = self.get_values(False)

        start_pose.position.x = float(pose[0])
        start_pose.position.y = float(pose[1])
        start_pose.position.z = float(pose[2])
        start_pose.orientation.w = float(pose[3])
        
        # setting the new joint goals
        # better would to set them as inverse kinematic
        
        self.move_group.set_pose_target(start_pose)
        
        print("Press Enter to go to starting pose")
        
        plan = self.move_group.go(wait=True)

        #get rid of any residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
    
    def go_to_goal_pose(self):
        
        print("jetzt in goal pose")
        pose_goal = geometry_msgs.msg.Pose()

        pose = self.get_values(True)

        pose_goal.position.x = float(pose[0])
        pose_goal.position.y = float(pose[1])
        pose_goal.position.z = float(pose[2])
        pose_goal.orientation.w = float(pose[3])

        self.move_group.set_pose_target(pose_goal)

        print("Press Enter to plan the path and go to pose goal")
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


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