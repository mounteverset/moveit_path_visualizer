#!/usr/bin/env python3

import os
import rospy
import rospkg
import io
import subprocess
from rqt_mypkg import path_planning_interface
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PySide2.QtCore import Qt, Slot, qWarning
from python_qt_binding.QtWidgets  import QFileDialog, QMessageBox
from python_qt_binding.QtWidgets  import QTableWidget, QTableWidgetItem

from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from geometry_msgs.msg import Pose, PoseArray
from pathlib import Path as Path
import signal

from visualization_msgs.msg import Marker, MarkerArray
from rqt_mypkg import path_planning_interface
from rqt_mypkg.msg import PathStatistics



class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

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
        self._widget.setObjectName('MyPluginUi')
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

        # Add slots to signal
        self._widget.pushButton_openPlanningScene.clicked.connect(self.on_pushButton_openPlanningScene_clicked)
        self._widget.pushButton_apply.clicked.connect(self.on_pushButton_apply_clicked)
        self._widget.pushButton_planPath.clicked.connect(self.on_pushButton_planPath_clicked)
        self._widget.pushButton_apply_planner.clicked.connect(self.on_pushButton_apply_planner_clicked)
        self._widget.ompl_display_checkBox.clicked.connect(self.on_checkBox_clicked)
        self._widget.chomp_display_checkBox.clicked.connect(self.on_checkBox_clicked)
        self._widget.stomp_display_checkBox.clicked.connect(self.on_checkBox_clicked)
        #self._widget.statisticsTable.clicked.connect(self.on_statistics_generated)
        #self._widget.pushButton.clicked.connect(self.pushButton_clicked)

        # Initialize a ROS subscriber
        
        sub = rospy.Subscriber('/statistics', PathStatistics, self.callback_subscriber)
        service = rospy.Service("/ompl_planner_id", Trigger, self.callback_service)
        self.active_motion_planner = None
        self.first_open = False

        # Add widget to the user interface
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

        # Part 1: Write the received data into the table
        print("We are here in the callback!")
        #print(str(msg.data))

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
            self._widget.ompl_display_checkBox.setEnabled(True)
            self._widget.ompl_display_checkBox.setChecked(True)

        elif self._widget.radioButton_CHOMP.isChecked() == True:
            column = 1
            self.chomp_pose_array = msg.eef_poses
            self.chomp_marker_array = msg.markers
            self._widget.chomp_display_checkBox.setEnabled(True)
            self._widget.chomp_display_checkBox.setChecked(True)

        elif self._widget.radioButton_STOMP.isChecked() == True:
            column = 2
            self.stomp_pose_array = msg.eef_poses
            self.stomp_marker_array = msg.markers
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
        
        # Part 2: Overwrite the eef_poses and markerarray attributes
        # done above

        # Part 3: check the right checkbox
        # done above

        # Part 4: display the paths from every checked path planner

        # Part 5: disable the pushButton_planPath for the next use
        self._widget.pushButton_planPath.setEnabled(False)
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        pass
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    
    @Slot()
    def on_pushButton_planPath_clicked(self):
        
        subprocess.Popen(["gnome-terminal", "-e", "rosrun rqt_mypkg planner.py"])
        # this funtion needs to write the current values from the doubleSpinBoxes into the QWidgetTable 
        # to see which paths have the same start and finish        

    @Slot()
    def on_pushButton_openPlanningScene_clicked(self):
        
        #fname = QFileDialog.getOpenFileName()
        subprocess.Popen(["gnome-terminal", "-e", "roslaunch rqt_mypkg demo_scene.launch"])
        #subprocess.call("roslaunch rqt_mypkg demo_scene.launch")
        
    #test
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
            #os.system("gnome-terminal 'source ~/ws_moveit/devel/setup.bash ; roslaunch fanuc_m710 demo.launch'")
        elif self._widget.radioButton_CHOMP.isChecked() == True:
            self.active_motion_planner = subprocess.Popen(["gnome-terminal", 
                                                            '--disable-factory', 
                                                            "-e", 
                                                            "roslaunch fanuc_m710 demo.launch pipeline:=chomp"],
                                                            preexec_fn=os.setpgrp)
            #os.system("gnome-terminal 'roslaunch fanuc_m710 demo.launch pipeline:=chomp'")
        elif self._widget.radioButton_STOMP.isChecked() == True:
            self.active_motion_planner = subprocess.Popen(["gnome-terminal", 
                                                            '--disable-factory',
                                                            "-e", 
                                                            "roslaunch fanuc_m710 demo.launch pipeline:=stomp"],
                                                            preexec_fn=os.setpgrp)

    # function needed to connect to an event from the gui:
    # when one of the checkboxes is getting checked or unchecked the marker array needs to update accordingly to the checkmarks
    # function replaces the one in path_planning_interface publish_marker_array

    @Slot()
    def on_checkBox_clicked(self):
        print("to be implemented")
        pass
    

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
    # @Slot()
    # def on_statistics_generated(self):
    #     planningObject = path_planning_interface.MoveGroupDefinedPath()
    #     statisticsObject = statistics.StatisticsDefinedPath()

    #     time = planningObject.getTime()
    #     length = statisticsObject.pathLength(path_planning_interface.eef_poses)









    #@Slot()
    #def pushButton_clicked(self):
        
    #    self.publisher = rospy.Publisher('visualization_marker',
    #                                                        Marker,
    #                                                        queue_size=5)
        
    #    marker = Marker()
    #    marker.header.frame_id = "world"
    #    marker.type = marker.SPHERE
    #    marker.action = marker.ADD
    #    marker.pose.position.x = 1
    #    marker.pose.position.y = 1
    #    marker.pose.position.z = 1
    #    marker.pose.orientation.x = 0.0
    #    marker.pose.orientation.y = 0.0
    #    marker.pose.orientation.z = 0.0
    #    marker.pose.orientation.w = 1.0
    #    marker.scale.x = 0.5
    #    marker.scale.y = 0.5
    #    marker.scale.z = 0.5
    #    marker.color.a = 1.0
    #    marker.color.r = 1.0
    #    marker.color.g = 1.0
    #    marker.color.b = 1.0
        
        
    #    self.publisher.publish(marker)
