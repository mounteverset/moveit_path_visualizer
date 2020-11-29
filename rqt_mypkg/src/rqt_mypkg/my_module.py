#!/usr/bin/env python3

import os
import rospy
import rospkg
import io
#from moveit_ros_planning import publish_scene_from_text
import xml.etree.ElementTree as xml


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PySide2.QtCore import Qt, Slot, qWarning
from PySide2.QtWidgets import QFileDialog, QMessageBox
from geometry_msgs.msg import Pose
from PyQt5.QtCore import Qt


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
        # Add widget to the user interface
        context.add_widget(self._widget)
        #os.system("roslaunch panda_moveit_config demo.launch")

        

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
        print("geklickt")
        alert = QMessageBox()
        alert.setText('You clicked the button!')
        alert.exec_()
        os.system("rosrun rqt_mypkg planner.py")

    @Slot()
    def on_pushButton_openPlanningScene_clicked(self):
        fname = QFileDialog.getOpenFileName()
        print (fname)
        os.system("roslaunch rqt_mypkg demo_scene.launch")
        
    @Slot()
    def on_pushButton_apply_clicked(self):
        print("clicked")
        alert = QMessageBox()
        alert.setText("Coordinates exported as .txt")
        alert.exec_()

        startingPose = Pose()
        goalPose = Pose()

        print (self._widget.doubleSpinBox_x1.value())
        startingPose.position.x = self._widget.doubleSpinBox_x1.value()
        startingPose.position.y = self._widget.doubleSpinBox_y1.value()
        startingPose.position.z = self._widget.doubleSpinBox_z1.value()
        startingPose.orientation.w = self._widget.doubleSpinBox_r1.value()

        
        with open("starting_pose.txt", "w") as f:
            f.write("{}\n{}\n{}\n{}".format(startingPose.position.x, 
                                            startingPose.position.y, 
                                            startingPose.position.z,
                                            startingPose.orientation.w))
        
        print("file erstellt")

        goalPose.position.x = self._widget.doubleSpinBox_x2.value()
        goalPose.position.y = self._widget.doubleSpinBox_y2.value()
        goalPose.position.z = self._widget.doubleSpinBox_z2.value()
        goalPose.orientation.w = self._widget.doubleSpinBox_r2.value()

        with open("goal_pose.txt", "w") as f:
            f.write("{}\n{}\n{}\n{}".format(goalPose.position.x, 
                                            goalPose.position.y, 
                                            goalPose.position.z,
                                            goalPose.orientation.w))
        
        print("file erstellt")
        
        self._widget.pushButton_apply.setEnabled(True)
        
        #startingPose.rotation.w
        #validate filename
        #if __name__ == "__main__":
        #    GenerateXML(self, "Coordinates1")   
    
        
    
