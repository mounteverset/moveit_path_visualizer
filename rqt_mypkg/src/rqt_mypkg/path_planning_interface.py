#!/usr/bin/env python3

import sys
import copy
from moveit_commander import move_group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from math import pi
from std_msgs.msg import String
import io
import shutil
import os
from pathlib import Path
from visualization_msgs.msg import Marker, MarkerArray


class MoveGroupDefinedPath(object):

    def __init__(self):
        super(MoveGroupDefinedPath, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        rospy.init_node('move_group_defined_path', anonymous=True)
        print("rospy Node started..")
        
        self.box_name = ""
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "fanuc_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

    def plan_path (self):
        goal_pose = geometry_msgs.msg.Pose()
        pose = self.get_values(False)
        goal_pose.position.x = float(pose[0])
        goal_pose.position.y = float(pose[1])
        goal_pose.position.z = float(pose[2])
        goal_pose.orientation.w = float(pose[3])   
        self.move_group.set_pose_target(goal_pose)
        motion_plan = self.move_group.plan()
        
        return motion_plan

    def go_to_starting_pose(self):
        
        print("Current joint_goals:")
        print (self.robot.get_current_state())

        start_pose = geometry_msgs.msg.Pose()
        
        
        #pause zum durchlesen der infos im terminal
        pose = self.get_values(True)

        start_pose.position.x = float(pose[0])
        start_pose.position.y = float(pose[1])
        start_pose.position.z = float(pose[2])
        start_pose.orientation.w = float(pose[3])
        
        # setting the new joint goals
        # better would to set them as inverse kinematic
        
        self.move_group.set_pose_target(start_pose)
        #self.move_group.plan()
        plan = self.move_group.go(wait=True)
        #self.move_group.execute(plan, wait=True)

        #get rid of any residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
  
    def go_to_goal_pose(self):
        
        pose_goal = geometry_msgs.msg.Pose()

        pose = self.get_values(True)

        pose_goal.position.x = float(pose[0])
        pose_goal.position.y = float(pose[1])
        pose_goal.position.z = float(pose[2])
        pose_goal.orientation.w = float(pose[3])

        self.move_group.set_pose_target(pose_goal)
        
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.delete_pose_files()

    def display_trajectory (self, plan):
                
        trajectory = moveit_msgs.msg.DisplayTrajectory()
        trajectory.trajectory_start = self.robot.get_current_state()
        trajectory.trajectory.append(plan)

        self.display_trajectory_publisher.publish(trajectory)

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def get_values(self, goal):

        homedir = str(Path.home())
        start_filepath = homedir + "/starting_pose.txt"
        goal_filepath = homedir + "/goal_pose.txt"

        if (goal == True):
            with open(start_filepath, "r") as f:
                pose = [line.rstrip() for line in f]

        elif (goal != True):
            with open(goal_filepath, "r") as f:
                pose = [line.rstrip() for line in f]
        else:
            return
        return pose   

    def delete_pose_files(self):

        homedir = str(Path.home())
        start_filepath = homedir + "/starting_pose.txt"
        goal_filepath = homedir + "/goal_pose.txt"

        if os.path.exists(start_filepath):
           os.remove(goal_filepath)
           os.remove(start_filepath)
        else:
           print("The file does not exist")

    def get_eef_poses(self, plan):

        eef_poses = []
        joint_goals = self.move_group.get_current_joint_values()
        
        # iterate through all of the joint positions saved in the plan
        for i in range(0, len(plan[1].joint_trajectory.points)):
            for j in range(0,6):
                joint_goals[j] = plan[1].joint_trajectory.points[i].positions[j]
            # set the goal pose of the robot to the joint positions
            # move the robot to the position
            self.move_group.go(joint_goals, wait=True)
            # save the position of the eef
            eef_poses.append(self.move_group.get_current_pose().pose)
        
        return eef_poses
        
        # either directly create a marker and append it to an array or save the poses in an array
    
    def display_eef_marker(self, eef_poses):

        publisher = rospy.Publisher('visualization_marker_array',
                                                            MarkerArray,
                                                            queue_size=1)
        
        markerArray = MarkerArray()

        for i in range(0, len(eef_poses)):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "link_base"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = eef_poses[i].position.x
            marker.pose.position.y = eef_poses[i].position.y
            marker.pose.position.z = eef_poses[i].position.z
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            markerArray.markers.append(marker)
        rospy.sleep(2)
        publisher.publish(markerArray)
        
    

           