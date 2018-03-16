#!/usr/bin/env python
# Lucas Walter
# Show a transform between two frames in rqt
import math
import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QAction, QLabel, QLineEdit, QMenu, QWidget
from std_msgs.msg import String
import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import TransformStamped
from tf import transformations

def _euler_from_quaternion_msg(quaternion):
    return transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    # transform3d went from from xyzw to wxyz
    # return euler_from_quaternion([quaternion.w, quaternion.x, quaternion.y, quaternion.z])

class TfEcho(Plugin):
    # do_update_label = QtCore.pyqtSignal(String)

    def __init__(self, context):
        super(TfEcho, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TfEcho')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_tf_echo'), 'resource', 'tf_echo.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TfEchoUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        cache_time = 10.0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_time))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TODO(lucasw) ros param
        self.source_frame = ""
        self.target_frame = ""
        self.label = {}
        self.label['current_time'] = self._widget.findChild(QLabel, 'current_time_label')
        self.label['transform_time'] = self._widget.findChild(QLabel, 'transform_time_label')
        for axis in ['x', 'y', 'z']:
            self.label['trans_' + axis] = self._widget.findChild(QLabel, 'trans_' + axis + '_label')
        for axis in ['x', 'y', 'z', 'w']:
            self.label['quat_' + axis] = self._widget.findChild(QLabel, 'quat_' + axis + '_label')
        for unit in ['rad', 'deg']:
            for axis in ['roll', 'pitch', 'yaw']:
                name = 'rot_' + axis + '_' + unit
                self.label[name] = self._widget.findChild(QLabel, name + '_label')
        self.source_line_edit = self._widget.findChild(QLineEdit, 'source_line_edit')
        self.target_line_edit = self._widget.findChild(QLineEdit, 'target_line_edit')

        self._widget.setContextMenuPolicy(QtCore.Qt.ActionsContextMenu)
        action = QAction("Show/hide Quaternion", self._widget)
        action.triggered.connect(self.hide_quaternion)
        self._widget.addAction(action)

        self.qt_timer = QTimer()
        self.qt_timer.start(100)
        self.qt_timer.timeout.connect(self.qt_update)

    def hide_quaternion(self):
        for axis in ['x', 'y', 'z', 'w']:
            if self.label['quat_' + axis].isHidden():
                self.label['quat_' + axis].show()
            else:
                self.label['quat_' + axis].hide()

    def qt_update(self):
        lookup_time = rospy.Time()
        cur_time = rospy.Time.now().to_sec()

        self.source_frame = self.source_line_edit.text()
        self.target_frame = self.target_line_edit.text()

        ts = None
        if self.source_frame != "" and self.target_frame != "":
            try:
                ts = self.tf_buffer.lookup_transform(self.source_frame,
                                                     self.target_frame,
                                                     lookup_time,
                                                     rospy.Duration(0.02))
            except tf2.LookupException as ex:
                msg = "At time {}, (current time {}) ".format(lookup_time.to_sec(), cur_time)
                rospy.logdebug(msg + str(ex))
            except tf2.ExtrapolationException as ex:
                msg = "(current time {}) ".format(cur_time)
                rospy.logdebug(msg + str(ex))
        # update the cur time in case lookup_transform was slow
        cur_time = rospy.Time.now().to_sec()
        self.label['current_time'].setText("{:1.2f}".format(cur_time))

        if ts is None:
            self.label['transform_time'].setStyleSheet('background-color: red')
            return

        self.label['transform_time'].setStyleSheet('')
        self.label['transform_time'].setText("{:1.2f}".format(ts.header.stamp.to_sec()))
        self.label['trans_x'].setText("{:1.3f}".format(ts.transform.translation.x))
        self.label['trans_y'].setText("{:1.3f}".format(ts.transform.translation.y))
        self.label['trans_z'].setText("{:1.3f}".format(ts.transform.translation.z))

        quat = ts.transform.rotation
        self.label['quat_x'].setText("{:1.3f}".format(quat.x))
        self.label['quat_y'].setText("{:1.3f}".format(quat.y))
        self.label['quat_z'].setText("{:1.3f}".format(quat.z))
        self.label['quat_w'].setText("{:1.3f}".format(quat.w))

        euler = _euler_from_quaternion_msg(quat)
        axes = ['roll', 'pitch', 'yaw']
        for i in range(3):
            self.label['rot_' + axes[i] + '_rad'].setText("{:1.3f}".format(euler[i]))
            self.label['rot_' + axes[i] + '_deg'].setText("{:1.3f}".format(math.degrees(euler[i])))

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO(lucasw) make rosparam override saved settings
        if instance_settings.contains('source_frame'):
            self.source_frame = instance_settings.value('source_frame')
            self.source_line_edit.setText(self.source_frame)
        if instance_settings.contains('target_frame'):
            self.target_frame = instance_settings.value('target_frame')
            self.target_line_edit.setText(self.target_frame)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('source_frame', self.source_frame)
        instance_settings.set_value('target_frame', self.target_frame)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
