#!/usr/bin/env python3

import yaml
import os
import rospy
import rospkg
import io
import subprocess
from rqt_mypkg import path_planning_interface
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PySide2.QtCore import Qt, Slot, QTranslator
from python_qt_binding.QtWidgets  import QFileDialog, QMessageBox
from python_qt_binding.QtWidgets  import QTableWidget, QTableWidgetItem

from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from geometry_msgs.msg import Pose, PoseArray
from pathlib import Path as Path
import signal
import random

from visualization_msgs.msg import Marker, MarkerArray
from rqt_mypkg import path_planning_interface, statistics
from rqt_mypkg.msg import PathStatistics



class MyPlugin(Plugin):
    """rqt plugin class which is used to create the plugin from a UI file generated with Qt Designer

    
    """
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PathPlanning')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('PathPlanningUi')
        self._widget.setWindowTitle("Path Planning with MoveIt!")
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Save the latest planned paths
        self.ompl_marker_array = MarkerArray()
        self.ompl_pose_array = PoseArray()
        self.chomp_marker_array = MarkerArray()
        self.chomp_pose_array = PoseArray()
        self.stomp_marker_array = MarkerArray()
        self.stomp_pose_array = PoseArray()

        # Add slots to signals
        self._widget.pushButton_openPlanningScene.clicked.connect(self.on_pushButton_openPlanningScene_clicked)
        self._widget.pushButton_apply.clicked.connect(self.on_pushButton_apply_clicked)
        self._widget.pushButton_planPath.clicked.connect(self.on_pushButton_planPath_clicked)
        self._widget.pushButton_apply_planner.clicked.connect(self.on_pushButton_apply_planner_clicked)
        self._widget.ompl_display_checkBox.clicked.connect(self.on_checkBox_clicked)
        self._widget.chomp_display_checkBox.clicked.connect(self.on_checkBox_clicked)
        self._widget.stomp_display_checkBox.clicked.connect(self.on_checkBox_clicked)
        self._widget.pushButton_planningScene_refresh.clicked.connect(self.on_pushButton_planningScene_refresh_clicked)
        self._widget.ompl_export_button.clicked.connect(self.on_ompl_export_clicked)
        self._widget.ompl_export_button.clicked.connect(self.on_ompl_export_clicked)
        self._widget.chomp_export_button.clicked.connect(self.on_chomp_export_clicked)
        self._widget.stomp_export_button.clicked.connect(self.on_stomp_export_clicked)

        # Initialize the planning scene tab        
        self.on_pushButton_planningScene_refresh_clicked()

        # Initialize a ROS subscriber to receive the statistics from the path planner        
        sub = rospy.Subscriber('/statistics', PathStatistics, self.callback_subscriber)

        # Initialize a ROS Service which is called by the path planner to receive 
        # information about the selected algorithm
        service = rospy.Service("/ompl_planner_id", Trigger, self.callback_service)

        # Attributes to handle the starting and shutting down of OMPL, CHOMP, STOMP in the program as their launch  
        # files must not be launched at the same time
        self.active_motion_planner = None
        self.first_open = False
        
        # Add widget to the rqt user interface
        context.add_widget(self._widget)

    def callback_service(self, request):
        print("service callback is called")
        response = TriggerResponse()
        if self._widget.radioButton_OMPL.isChecked()== True:
            response.success = True
            response.message = self._widget.comboBox_ompl.currentText()
        else:
            response.success = False
            response.message = ""
        return response


    # things that need to happen in this callback:
    # all of the data from the received message is written into the correct column and cell
    # for this the current used planner has to determined by checking which radio buttons is checked
    # also the checkbox at the bottom of the planner has to be automatically checked so that the path is visible by default
    # the marker_array and the eef_poses from the message need to be saved in this program as object attributes in order to make them kind of persistent and accessible
    # each motion_planner has their own respective set of variables but they can only store the values from the last plan that was executed with the motion planner
    # also the plan_path_pushButton on the bottom of the gui needs to be disabled, because there are no points in a txt to read from right now
    # also there needs to be a check whether some data is already in the columns above, bc if there isn't then the checkbox shouldnt be enabled
    def callback_subscriber(self, msg):

        print("rqt callback")
        if msg.planning_time == 0 and msg.path_length == 0 and msg.max_acceleration == 0:
            print("No solution for the inverse kinematic was found. \nPlease use different points closer to the robot.")

        elif msg.planning_time == 1 and msg.path_length == 1 and msg.max_acceleration == 1:
            print("No motion plan was found in time for the given points.\nTry calculating it again or use different points.\nIncrease the allowed time to find a solution.")

        else:
            # ompl column number = 0
            # chomp column number = 1
            # stomp column number = 2
            # start pose row = 0
            # goal pose row = 1
            # planning time row = 2
            # execution time row = 3
            # path length = 4
            # max joint accel = 5
            column = -1

            if self._widget.radioButton_OMPL.isChecked()== True:
                column = 0

                self.ompl_pose_array = msg.eef_poses
                self.ompl_marker_array = msg.markers

                for marker in self.ompl_marker_array.markers:
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                rnd = random.randint(0,len(self.ompl_marker_array.markers)-1)
                text_marker = self.create_text_marker("OMPL", 
                                                        self.ompl_marker_array.markers[len(self.ompl_marker_array.markers)-1].id, 
                                                        self.ompl_marker_array.markers[rnd].pose)       
                self.ompl_marker_array.markers.append(text_marker)
                
                self._widget.ompl_display_checkBox.setEnabled(True)
                self._widget.ompl_display_checkBox.setChecked(True)


            elif self._widget.radioButton_CHOMP.isChecked() == True:
                if self._widget.checkBox_chomp_ompl_prep.isChecked() == True:
                    column = 1
                    self.chomp_pose_array = msg.eef_poses
                    self.chomp_marker_array = msg.markers
                    
                    for marker in self.chomp_marker_array.markers:
                        if marker.id % 2 == 0:
                            marker.id += 150
                            marker.color.a = 1.0
                            marker.color.r = 0.0
                            marker.color.g = 0.0
                            marker.color.b = 1.0
                        else:
                            marker.color.a = 1.0
                            marker.color.r = 1.0
                            marker.color.g = 1.0
                            marker.color.b = 1.0
                    rnd = random.randint(0,len(self.chomp_marker_array.markers)-1)
                    text_marker = self.create_text_marker("CHOMP (processed)", 
                                                            self.chomp_marker_array.markers[len(self.chomp_marker_array.markers)-1].id, 
                                                            self.chomp_marker_array.markers[rnd].pose) 
                    text_marker.color.a = 1.0
                    text_marker.color.r = 0.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 1.0        
                    self.chomp_marker_array.markers.append(text_marker)

                    self._widget.chomp_display_checkBox.setEnabled(True)
                    self._widget.chomp_display_checkBox.setChecked(True)

                else:
                    column = 1
                    self.chomp_pose_array = msg.eef_poses
                    self.chomp_marker_array = msg.markers
                    
                    for marker in self.chomp_marker_array.markers:
                        marker.id += 150
                        marker.color.a = 1.0
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 1.0
                    rnd = random.randint(0,len(self.chomp_marker_array.markers)-1)
                    text_marker = self.create_text_marker("CHOMP", 
                                                            self.chomp_marker_array.markers[len(self.chomp_marker_array.markers)-1].id, 
                                                            self.chomp_marker_array.markers[rnd].pose) 
                    text_marker.color.a = 1.0
                    text_marker.color.r = 0.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 1.0        
                    self.chomp_marker_array.markers.append(text_marker)

                    self._widget.chomp_display_checkBox.setEnabled(True)
                    self._widget.chomp_display_checkBox.setChecked(True)


            elif self._widget.radioButton_STOMP.isChecked() == True:
                if self._widget.checkBox_stomp_chomp_postp.isChecked() == True:
                    column = 2
                    self.stomp_pose_array = msg.eef_poses
                    self.stomp_marker_array = msg.markers

                    for marker in self.stomp_marker_array.markers:
                        marker.id += 350
                        if marker.id % 2 == 0:
                            marker.color.a = 1.0
                            marker.color.r = 1.0
                            marker.color.g = 0.0
                            marker.color.b = 0.0
                        else:
                            marker.color.a = 1.0
                            marker.color.r = 0.0
                            marker.color.g = 0.0
                            marker.color.b = 1.0 

                    rnd = random.randint(0,len(self.stomp_marker_array.markers)-1)
                    text_marker = self.create_text_marker("STOMP (processed)", 
                                                        self.stomp_marker_array.markers[len(self.stomp_marker_array.markers)-1].id, 
                                                        self.stomp_marker_array.markers[rnd].pose)  
                    text_marker.color.a = 1.0
                    text_marker.color.r = 1.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 0.0     
                    self.stomp_marker_array.markers.append(text_marker)

                    self._widget.stomp_display_checkBox.setEnabled(True)
                    self._widget.stomp_display_checkBox.setChecked(True)
                
                else:
                    column = 2
                    self.stomp_pose_array = msg.eef_poses
                    self.stomp_marker_array = msg.markers

                    for marker in self.stomp_marker_array.markers:
                        marker.id += 350
                        marker.color.a = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0

                    rnd = random.randint(0,len(self.stomp_marker_array.markers)-1)
                    text_marker = self.create_text_marker("STOMP", 
                                                        self.stomp_marker_array.markers[len(self.stomp_marker_array.markers)-1].id, 
                                                        self.stomp_marker_array.markers[rnd].pose)  
                    text_marker.color.a = 1.0
                    text_marker.color.r = 1.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 0.0     
                    self.stomp_marker_array.markers.append(text_marker)

                    self._widget.stomp_display_checkBox.setEnabled(True)
                    self._widget.stomp_display_checkBox.setChecked(True)


            if column == -1:
                print("No active motion planner found...")
            
            start_values = self.get_start_values()
            goal_values = self.get_goal_values()
            start_table_item = QTableWidgetItem("[{}]".format("|".join(start_values)))
            self._widget.statisticsTable.setItem(0,column, start_table_item)
            goal_table_item = QTableWidgetItem("[{}]".format("|".join(goal_values)))
            self._widget.statisticsTable.setItem(1,column, goal_table_item)
            planning_time = QTableWidgetItem(str(round(msg.planning_time,5)))       
            self._widget.statisticsTable.setItem(2,column, planning_time)
            execution_time = QTableWidgetItem(str(round(msg.execution_time,5))) 
            self._widget.statisticsTable.setItem(3,column, execution_time)
            path_length = QTableWidgetItem(str(round(msg.path_length,5)))
            self._widget.statisticsTable.setItem(4,column, path_length)
            joint_accel = planning_time = QTableWidgetItem(str(round(msg.max_acceleration,5))) 
            self._widget.statisticsTable.setItem(5,column, joint_accel)

            # display the paths from every checked path planner
            self.publish_marker_array()

            # Part 5: disable the pushButton_planPath for the next use
            self._widget.pushButton_planPath.setEnabled(False)
    
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def trigger_configuration(self):
        pass
    
    @Slot()
    def on_pushButton_planPath_clicked(self):
        
        subprocess.Popen(["gnome-terminal", "-e", "rosrun rqt_mypkg planner.py"])       

    @Slot()
    def on_pushButton_openPlanningScene_clicked(self):
        
        current_item = self._widget.planningScene_listWidget.currentItem()
        if current_item.text() != "":
            working_dir = str(Path(__file__).parent.absolute())
            parent_dir_list = working_dir.split("/")[:-2]
            parent_dir = "/".join(parent_dir_list)
            scene_dir = os.path.join(parent_dir, "scenes")
            filepath = os.path.join(scene_dir, current_item.text())
            subprocess.Popen(["gnome-terminal", "-e", "roslaunch rqt_mypkg scene.launch scene_file:={}".format(filepath)])
        else:
            print("No planning scene selected.")

        
    @Slot()
    def on_pushButton_apply_clicked(self):
        
        startingPose = Pose()
        goalPose = Pose()

        startingPose.position.x = self._widget.doubleSpinBox_x1.value()
        startingPose.position.y = self._widget.doubleSpinBox_y1.value()
        startingPose.position.z = self._widget.doubleSpinBox_z1.value()
        startingPose.orientation.w = self._widget.doubleSpinBox_r1.value()
        homedir = str(Path.home())
        start_filepath = homedir + "/starting_pose.txt"
        with open(start_filepath, "w") as f:
            f.write("{}\n{}\n{}\n{}".format(startingPose.position.x, 
                                            startingPose.position.y, 
                                            startingPose.position.z,
                                            startingPose.orientation.w))
    
        goalPose.position.x = self._widget.doubleSpinBox_x2.value()
        goalPose.position.y = self._widget.doubleSpinBox_y2.value()
        goalPose.position.z = self._widget.doubleSpinBox_z2.value()
        goalPose.orientation.w = self._widget.doubleSpinBox_r2.value()

        goal_filepath = homedir + "/goal_pose.txt"
        with open(goal_filepath, "w") as f:
            f.write("{}\n{}\n{}\n{}".format(goalPose.position.x, 
                                            goalPose.position.y, 
                                            goalPose.position.z,
                                            goalPose.orientation.w))
        
        self._widget.pushButton_planPath.setEnabled(True)

                
    @Slot()
    def on_pushButton_apply_planner_clicked(self):
        
        self._widget.pushButton_openPlanningScene.setEnabled(True)

        if self.active_motion_planner != None:
            os.killpg(self.active_motion_planner.pid, signal.SIGINT)
            print("Trying to terminate old motion planner...")

        if self._widget.radioButton_OMPL.isChecked()== True:
            self.active_motion_planner = subprocess.Popen(["gnome-terminal", 
                                                            '--disable-factory', 
                                                            "-e", 
                                                            "roslaunch fanuc_m710 demo.launch"], 
                                                            preexec_fn=os.setpgrp)

        elif self._widget.radioButton_CHOMP.isChecked() == True:
            if self._widget.checkBox_chomp_ompl_prep.isChecked() == True:
                self.active_motion_planner = subprocess.Popen(["gnome-terminal", 
                                                            '--disable-factory', 
                                                            "-e", 
                                                            "roslaunch fanuc_m710 demo.launch pipeline:=chomp_ompl_prep"],
                                                            preexec_fn=os.setpgrp)
            else:
                self.active_motion_planner = subprocess.Popen(["gnome-terminal", 
                                                            '--disable-factory', 
                                                            "-e", 
                                                            "roslaunch fanuc_m710 demo.launch pipeline:=chomp"],
                                                            preexec_fn=os.setpgrp)

        elif self._widget.radioButton_STOMP.isChecked() == True:
            if self._widget.checkBox_stomp_chomp_postp.isChecked() == True:
                self.active_motion_planner = subprocess.Popen(["gnome-terminal", 
                                                            '--disable-factory',
                                                            "-e", 
                                                            "roslaunch fanuc_m710 demo.launch pipeline:=stomp_chomp_postp"],
                                                            preexec_fn=os.setpgrp)
            else:
                self.active_motion_planner = subprocess.Popen(["gnome-terminal", 
                                                            '--disable-factory',
                                                            "-e", 
                                                            "roslaunch fanuc_m710 demo.launch pipeline:=stomp"],
                                                            preexec_fn=os.setpgrp)

    @Slot()
    def on_checkBox_clicked(self):
        self.publish_marker_array()
        
    @Slot()
    def on_ompl_export_clicked(self):
        dialog = QFileDialog()
        directory, _filter = dialog.getSaveFileName(None, 'Choose a file name')
        print(directory)
        try:
            with open(directory, 'w') as file_save:
                yaml.dump(self.ompl_pose_array, file_save, default_flow_style=False)
        except FileNotFoundError:
            pass

    @Slot()
    def on_chomp_export_clicked(self):
        dialog = QFileDialog()
        directory, _filter = dialog.getSaveFileName(None, 'Choose a file name')
        print(directory)
        try:
            with open(directory, 'w') as file_save:
                yaml.dump(self.ompl_pose_array, file_save, default_flow_style=False)
        except FileNotFoundError:
            pass

    @Slot()
    def on_stomp_export_clicked(self):
        dialog = QFileDialog()
        directory, _filter = dialog.getSaveFileName(None, 'Choose a file name')
        print(directory)
        try:
            with open(directory, 'w') as file_save:
                yaml.dump(self.ompl_pose_array, file_save, default_flow_style=False)
        except FileNotFoundError:
            pass

    def publish_marker_array(self):

        publisher = rospy.Publisher('visualization_marker_array',
                                    MarkerArray,
                                    queue_size=1)

        markerArray = MarkerArray()
        for i in range(0,500):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "link_base"
            marker.action = marker.DELETE
            markerArray.markers.append(marker)
        rospy.sleep(1)
        publisher.publish(markerArray)
        markerArray.markers.clear()

        if self._widget.ompl_display_checkBox.isChecked():
            markerArray.markers.extend(self.ompl_marker_array.markers)
        if self._widget.chomp_display_checkBox.isChecked():
            markerArray.markers.extend(self.chomp_marker_array.markers)
        if self._widget.stomp_display_checkBox.isChecked():
            markerArray.markers.extend(self.stomp_marker_array.markers)

        rospy.sleep(1)
        publisher.publish(markerArray)

    def get_start_values(self):
        x = str(round(self._widget.doubleSpinBox_x1.value(),1))
        y = str(round(self._widget.doubleSpinBox_y1.value(),1))
        z = str(round(self._widget.doubleSpinBox_z1.value(),1))
        #w = str(self._widget.doubleSpinBox_r1.value())
        return x,y,z

    def get_goal_values(self):

        x = str(round(self._widget.doubleSpinBox_x2.value(),1))
        y = str(round(self._widget.doubleSpinBox_y2.value(),1))
        z = str(round(self._widget.doubleSpinBox_z2.value(),1))
        #w = str(self._widget.doubleSpinBox_r2.value())
        return x,y,z

    def create_text_marker(self, text, id, pose):

        marker = Marker()
        marker.id = id + 1
        marker.header.frame_id = "link_base"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = text
        marker.action = marker.ADD
        marker.pose.position.x = pose.position.x 
        marker.pose.position.y = pose.position.y 
        marker.pose.position.z = pose.position.z + 0.3
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        
        return marker

    def on_pushButton_planningScene_refresh_clicked(self):
        self._widget.planningScene_listWidget.clear()
        working_dir = str(Path(__file__).parent.absolute())
        parent_dir_list = working_dir.split("/")[:-2]
        parent_dir = "/".join(parent_dir_list)
        scene_dir = os.path.join(parent_dir, "scenes")
        dir_content = os.listdir(scene_dir)
        if len(dir_content) > 0:
            for scene in dir_content:
                if "." in scene:
                    self._widget.planningScene_listWidget.addItem(scene)
