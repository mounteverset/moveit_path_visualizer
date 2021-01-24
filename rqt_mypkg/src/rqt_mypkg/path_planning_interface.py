#!/usr/bin/env python3

import sys
import copy
from moveit_commander import move_group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from math import pi
from rospy import service
from rospy.topics import Publisher
from std_msgs.msg import String
import io
import shutil
#used to convert the points from the gui in a valid message for ros
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
#used to read out the start points
import os
from pathlib import Path
#used for publishing the planned path from start to goal
from visualization_msgs.msg import Marker, MarkerArray
#used to make a service request 
from std_srvs.srv import TriggerRequest, Trigger
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from rqt_mypkg.msg import PathStatistics

class MoveGroupDefinedPath(object):

    def __init__(self):
        super(MoveGroupDefinedPath, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
                                                            
        self.statistics_publisher = rospy.Publisher("PathStatistics", 
                                                    moveit_msgs.msg.RobotTrajectory, 
                                                    queue_size=1)
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
        self.starting_pose = self.get_values(False)
        self.goal_pose = self.get_values(True)
        self.starting_joint_goal = self.get_inverse_kinematic(False)
        self.goal_joint_goal = self.get_inverse_kinematic(True)

        self.planner_id = self.get_planner_id()
        if self.planner_id != "":
            self.move_group.set_planner_id(self.planner_id)
        
  

    def go_to_starting_joint_goal(self):

        print("Current joint_positions:")
        print (self.robot.get_current_state())
               
        #self.move_group.set_pose_target(self.starting_joint_goal)
        self.move_group.set_start_state(self.robot.get_current_state())
        self.move_group.go(self.starting_joint_goal, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def plan_path_from_to_start_with_joint_goal(self):
        self.move_group.set_start_state(self.robot.get_current_state())
        motion_plan = self.move_group.plan(self.starting_joint_goal)    
        return motion_plan

    def plan_path_from_joint_goal(self):
        
        self.move_group.set_start_state(self.robot.get_current_state())
        #self.move_group.set_pose_target(self.goal_joint_goal)
        motion_plan = self.move_group.plan(self.goal_joint_goal)        
        return motion_plan     

    def plan_path_from_pose (self):
                
        self.move_group.set_pose_target(self.goal_pose)
        motion_plan = self.move_group.plan()        
        return motion_plan

    def go_to_starting_pose(self):
        
        print("Current joint_positions:")
        print (self.robot.get_current_state())
               
        self.move_group.set_pose_target(self.starting_pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
  
    def go_to_goal_pose(self):
        
        self.move_group.set_pose_target(self.goal_joint_goal)        
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

    #if goal == True -> goal pose
    def get_values(self, goal):

        homedir = str(Path.home())
        start_filepath = homedir + "/starting_pose.txt"
        goal_filepath = homedir + "/goal_pose.txt"
        pose = Pose()
        
        if (goal == False):
            with open(start_filepath, "r") as f:
                points = [line.rstrip() for line in f]
            
        elif (goal == True):
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
            self.move_group.set_start_state(self.robot.get_current_state())
            self.move_group.execute(robot_trajectory, wait=True)
            #früher mit move_group.go aber es wurde immer wieder geplant
            prev = i
            # save the position of the eef
            eef_poses.append(self.move_group.get_current_pose().pose)
        
        return eef_poses
        
        # either directly create a marker and append it to an array or save the poses in an array
    
    def create_eef_marker(self, eef_poses):

        # publisher = rospy.Publisher('visualization_marker_array',
        #                                                     MarkerArray,
        #                                                     queue_size=1)

        markerArray = MarkerArray()
        # for i in range(0,100):
        #     marker = Marker()
        #     marker.id = i
        #     marker.header.frame_id = "link_base"
        #     marker.action = marker.DELETE
        #     markerArray.markers.append(marker)
        # rospy.sleep(1)
        # publisher.publish(markerArray)

        # markerArray.markers.clear()

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
        
        # marker = Marker()
        # marker.id = len(markerArray.markers)
        # marker.header.frame_id = "link_base"
        # marker.type = marker.TEXT_VIEW_FACING
        # marker.text = "OMPL Pfad"
        # marker.action = marker.ADD
        # marker.pose.position.x = markerArray.markers[2].pose.position.x 
        # marker.pose.position.y = markerArray.markers[2].pose.position.y 
        # marker.pose.position.z = markerArray.markers[2].pose.position.z + 0.1
        # marker.scale.z = 0.2
        # marker.color.a = 1.0
        # marker.color.r = 1.0
        # marker.color.g = 1.0
        # marker.color.b = 1.0
        # markerArray.markers.append(marker)

        # rospy.sleep(1)
        # publisher.publish(markerArray)

        return markerArray

    # wenn goal als True übergeben wird, dann wird die inverse kinematik für die goal_pose berechnet
    def get_inverse_kinematic(self, goal):
        
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
        if goal == True:
            pose.pose = self.goal_pose 
        elif goal == False:
            pose.pose = self.starting_pose

        request.ik_request.pose_stamped = pose

        response = compute_ik_service(request)

        return response.solution.joint_state

    # To Do: create a PoseArray() Message and insert the values from eef_poses
    # create a PathStatistics() Msg and insert all of the values into it

    def publish_statistics(self,path_length, markers, planning_time, max_accel, eef_poses, exec_time):
        
        publisher = rospy.Publisher('/statistics', PathStatistics, queue_size=1)
        msg = PathStatistics()
        msg.path_length = path_length
        msg.planning_time = planning_time
        msg.max_acceleration = max_accel
        msg.markers = markers
        msg.execution_time = exec_time

        pose_array = PoseArray()
        pose_array.poses = eef_poses

        msg.eef_poses = pose_array
        
        rospy.sleep(1)
        publisher.publish(msg)

    def get_planner_id(self):

        service_client = rospy.ServiceProxy("ompl_planner_id", Trigger)
        try:
            rospy.wait_for_service("/ompl_planner_id", 5)
        except rospy.exceptions.ROSException as exec:
            print(exec)
        
        response = service_client(TriggerRequest())
        if response.success == True:
            return response.message
        else:
            return ""
