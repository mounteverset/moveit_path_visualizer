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
#used to convert the points from the gui in a valid message for ros
from geometry_msgs.msg import Pose, PoseStamped
#used to read out the start points
import os
from pathlib import Path
#used for publishing the planned path from start to goal
from visualization_msgs.msg import Marker, MarkerArray
#used to make a service request 
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK

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
        self.move_group.set_goal_joint_tolerance(0.005)
        self.move_group.set_goal_orientation_tolerance(0.05)
        self.move_group.set_goal_position_tolerance(0.05)
        self.move_group.set_goal_tolerance(0.05)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.starting_pose = self.get_values(True) #evtl automatische konvertierung in joint poses damit man try
        self.goal_pose = self.get_values(False)
         

    def plan_path_from_pose (self):
                
        self.move_group.set_pose_target(self.goal_pose)
        motion_plan = self.move_group.plan()
        
        return motion_plan

    def go_to_starting_pose(self):
        
        print("Current joint_goals:")
        print (self.robot.get_current_state())
        #start_pose = geometry_msgs.msg.Pose()                
        #pause zum durchlesen der infos im terminal
        #pose = self.get_values(True)
        #start_pose.position.x = float(pose[0])
        #start_pose.position.y = float(pose[1])
        #start_pose.position.z = float(pose[2])
        #start_pose.orientation.w = float(pose[3])        
        # setting the new joint goals
        # better would to set them as inverse kinematic        
        self.move_group.set_pose_target(self.starting_pose)
        #self.move_group.plan()
        plan = self.move_group.go(wait=True)
        #self.move_group.execute(plan, wait=True)
        #get rid of any residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
  
    def go_to_goal_pose(self):
        
        #pose_goal = geometry_msgs.msg.Pose()

        #pose = self.get_values(True)

        #pose_goal.position.x = float(pose[0])
        #pose_goal.position.y = float(pose[1])
        #pose_goal.position.z = float(pose[2])
        #pose_goal.orientation.w = float(pose[3])
        #self.starting_pose = pose_goal
        self.move_group.set_pose_target(self.goal_pose)        
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.delete_pose_files()

    def go_to_joint_space_goal(self, joint_goal):

        group_variable_values = move_group.get_current_joint_values()
        print("Current joint space goals: ")
        print(group_variable_values)

        #group_variable_values[0]=1.0
        self.move_group.set_joint_value_target(joint_goal)
        plan = self.move_group.go(wait=True)

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
        pose = Pose()
        #wenn true übergeben wird dann holt er sich die startpunkte
        if (goal == True):
            with open(start_filepath, "r") as f:
                points = [line.rstrip() for line in f]
            
        elif (goal != True):
            with open(goal_filepath, "r") as f:
                points = [line.rstrip() for line in f]
        else:
            return
        pose.position.x = float(points[0])
        pose.position.y = float(points[1])
        pose.position.z = float(points[2])
        pose.orientation.w = float(points[3])
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
        #joint_goals = self.move_group.get_current_joint_values()
        robot_trajectory = moveit_msgs.msg.RobotTrajectory()
        robot_trajectory.joint_trajectory.header = plan[1].joint_trajectory.header
        robot_trajectory.joint_trajectory.joint_names = plan[1].joint_trajectory.joint_names
        prev = 0
        # iterate through all of the joint positions saved in the plan
        for i in range(1, len(plan[1].joint_trajectory.points)):
            #for j in range(0,6):
            #    joint_goals[j] = plan[1].joint_trajectory.points[i].positions[j]
            # set the goal pose of the robot to the joint positions
            # move the robot to the position

            #ich möchte den robottrajectory von einem zum nächsten schritt aus dem plan
            robot_trajectory.joint_trajectory.points = plan[1].joint_trajectory.points[prev:i]
            #fake_plan = (True, robot_trajectory, plan[2], plan[3])
            self.move_group.execute(robot_trajectory, wait=True)
            #früher mit move_group.go aber es wurde immer wieder geplant
            prev = i
            # save the position of the eef
            eef_poses.append(self.move_group.get_current_pose().pose)
        
        return eef_poses
        
        # either directly create a marker and append it to an array or save the poses in an array
    
    def display_eef_marker(self, eef_poses):

        publisher = rospy.Publisher('visualization_marker_array',
                                                            MarkerArray,
                                                            queue_size=1)
        default_pose = Pose()
        default_pose.position.x = 0.0
        default_pose.position.y = 0.0
        default_pose.position.z = 0.0

        markerArray = MarkerArray()
        for i in range(0,100):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "link_base"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            #marker.lifetime = 2
            marker.pose.position.x = default_pose.position.x
            marker.pose.position.y = default_pose.position.y
            marker.pose.position.z = default_pose.position.z
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            markerArray.markers.append(marker)
        rospy.sleep(1)
        publisher.publish(markerArray)

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
        rospy.sleep(1)
        publisher.publish(markerArray)
        
    def get_inverse_kinematic(self):
        
        compute_ik_service = rospy.ServiceProxy("compute_ik", GetPositionIK)
        try:
            rospy.wait_for_service("compute_ik", 5)
        except rospy.exceptions.ROSException as exec:
            print(exec)
        
        request = GetPositionIKRequest()
        request.ik_request.group_name = "fanuc_arm"
        request.ik_request.robot_state = self.robot.get_current_state()
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = "link_6"
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "link_base"
        pose.pose = self.goal_pose #Diese zeile noch anpassen, da man eine pose übergeben möchte 
        request.ik_request.pose_stamped = pose

        response = compute_ik_service(request)

        return response
    

  

        
    


           