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

from std_msgs.msg import Int32 
from geometry_msgs.msg import Pose
from pathlib import Path as Path
import signal

from visualization_msgs.msg import Marker, MarkerArray
from rqt_mypkg import path_planning_interface



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

        # Add slots to signal
        self._widget.pushButton_openPlanningScene.clicked.connect(self.on_pushButton_openPlanningScene_clicked)
        self._widget.pushButton_apply.clicked.connect(self.on_pushButton_apply_clicked)
        self._widget.pushButton_planPath.clicked.connect(self.on_pushButton_planPath_clicked)
        self._widget.pushButton_apply_planner.clicked.connect(self.on_pushButton_apply_planner_clicked)
        #self._widget.statisticsTable.clicked.connect(self.on_statistics_generated)
        #self._widget.pushButton.clicked.connect(self.pushButton_clicked)

        # Initialize a ROS subscriber
        #rospy.init_node('statistics_subscriber', anonymous=True)
        sub = rospy.Subscriber('counter', Int32, self.callback_subscriber)

        self.active_motion_planner = None
        self.first_open = False

        # Add widget to the user interface
        context.add_widget(self._widget)

    def callback_subscriber(self, msg):
        print("We are here in the callback")
        new_item = QTableWidgetItem(msg.data)
        #new_item.set
        self._widget.statisticsTable.setItem(2,2,new_item)

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

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    
    @Slot()
    def on_pushButton_planPath_clicked(self):

        #if abfragen welche der radio buttons checked ist und dann den parameter weitergeben
        #  
        subprocess.Popen(["gnome-terminal", "-e", "rosrun rqt_mypkg planner.py"])
        #subprocess.call("rosrun rqt_mypkg planner.py")
        

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
            #os.system("gnome-terminal sawwqewq'roslaunch fanuc_m710 demo.launch pipeline:=stomp'")
    

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
