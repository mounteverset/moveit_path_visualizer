import os
import rospy
import rospkg
import xml.etree.ElementTree as xml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Qt, Slot, qWarning
from PySide2.QtWidgets import QFileDialog, QMessageBox


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
        self._widget.pushButton_planPath.clicked.connect(self.on_pushButton_planPath_clicked)
        self._widget.pushButton_openPlanningScene.clicked.connect(self.on_pushButton_openPlanningScene_clicked)
        self._widget.pushbutton_apply.clicked.connect(self.on_pushButton_apply_clicked)
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
    
    """
    def GenerateXML(self, fileName):
        root = xml.Element("Coordinates")
        child1 = xml.Element("Startpoint")
        child2 = xml.Element("Endpoint")
        root.append(child1)
        root.append(child2)
        
        x1 = xml.SubElement(child1, "X-Coordinate")
        y1 = xml.SubElement(child1, "Y-Coordinate")
        z1 = xml.SubElement(child1, "Z-Coordinate")
        
        x2 = xml.SubElement(child2, "X-Coordinate")
        y2 = xml.SubElement(child2, "Y-Coordinate")
        z2 = xml.SubElement(child2, "Z-Coordinate")
    """

        

        tree = xml.ElementTree(root)
        with open(fileName, "wb") as files:
            tree.write(files)
    


    @Slot()
    def on_pushButton_planPath_clicked(self):
        print("geklickt")
        alert = QMessageBox()
        alert.setText('You clicked the button!')
        alert.exec_()
        os.system("rosrun moveit_tutorials move_group_python_interface_tutorial.py")

    @Slot()
    def on_pushButton_openPlanningScene_clicked(self):
        fname = QFileDialog.getOpenFileName()
        print (fname)
        
    @Slot()
    def on_pushButton_apply_clicked(self):
        print("clicked")
        alert = QMessageBox()
        alert.setText("Coordinates exported as .xml")
        alert.exec_()
        #validate filename
        #if __name__ == "__main__":
        #    GenerateXML(self, "Coordinates1")   
    
