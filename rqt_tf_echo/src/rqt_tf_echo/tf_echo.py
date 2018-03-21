#!/usr/bin/env python
# Lucas Walter
# Show a transform between two frames in rqt
import math
import os
import rospkg
import rospy

from functools import partial
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
        # TODO(lucasw) these aren't working
        # parser.add_argument("source_frame", default='')  # parent
        # parser.add_argument("target_frame", default='')  # child
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self.source_frame = ''
        self.target_frame = ''
        # self.source_frame = self.args.source_frame
        # self.target_frame = self.args.target_frame

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
        for axis in ['x', 'y', 'z', 'label']:
            self.label['trans_' + axis] = self._widget.findChild(QLabel, 'trans_' + axis + '_label')
            self.label['trans_' + axis + '_2'] = self._widget.findChild(QLabel, 'trans_' + axis + '_label_2')
        for axis in ['x', 'y', 'z', 'w']:
            self.label['quat_' + axis] = self._widget.findChild(QLabel, 'quat_' + axis + '_label')
            self.label['quat_' + axis + '_2'] = self._widget.findChild(QLabel, 'quat_' + axis + '_label_2')
        for unit in ['rad', 'deg']:
            for axis in ['roll', 'pitch', 'yaw', 'label']:
                name = 'rot_' + axis + '_' + unit
                self.label[name] = self._widget.findChild(QLabel, name + '_label')
                self.label[name + '_2'] = self._widget.findChild(QLabel, name + '_label_2')
        self.label['source'] = self._widget.findChild(QLineEdit, 'source_line_edit')
        self.label['target'] = self._widget.findChild(QLineEdit, 'target_line_edit')

        self._widget.setContextMenuPolicy(QtCore.Qt.ActionsContextMenu)
        self.actions = {}
        self.setup_menu()

        self.qt_timer = QTimer()
        self.qt_timer.start(100)
        self.qt_timer.timeout.connect(self.qt_update)

    def add_menu_item(self, menu_name, labels):
        menu_text = menu_name
        if self.label[labels[0]].isHidden():
            menu_text = "Show " + menu_text
        else:
            menu_text = "Hide " + menu_text

        action = QAction(menu_text, self._widget)
        action.triggered.connect(partial(self.toggle_labels, labels))
        self._widget.addAction(action)

        self.actions[menu_name] = action

    def setup_menu(self):
        for key in self.actions.keys():
            self._widget.removeAction(self.actions[key])

        self.add_menu_item("transform time", ["transform_time"])
        self.add_menu_item("current time", ["current_time"])
        self.add_menu_item("source/parent frame", ["source"])
        self.add_menu_item("target/child frame", ["target"])
        self.add_menu_item("translation", ["trans_x", "trans_y", "trans_z", "trans_label"])
        self.add_menu_item("rotation quaternion", ["quat_x", "quat_y", "quat_z", "quat_w"])
        self.add_menu_item("rotation (radians)",
                           ["rot_roll_rad", "rot_pitch_rad", "rot_yaw_rad", "rot_label_rad"])
        self.add_menu_item("rotation (degrees)",
                           ["rot_roll_deg", "rot_pitch_deg", "rot_yaw_deg", "rot_label_deg"])

    def toggle(self, name):
        if self.label[name].isHidden():
            self.label[name].show()
            if name + '_2' in self.label.keys():
                self.label[name + '_2'].show()
        else:
            self.label[name].hide()
            if name + '_2' in self.label.keys():
                self.label[name + '_2'].hide()

    def toggle_labels(self, labels):
        for label in labels:
            self.toggle(label)
        self.setup_menu()

    def qt_update(self):
        lookup_time = rospy.Time()
        cur_time = rospy.Time.now().to_sec()

        self.source_frame = self.label['source'].text()
        self.target_frame = self.label['target'].text()

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
            # TODO(lucasw) need a dedicated status label in case the user
            # hides this?
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

    def get_param_setting(self, name, label_name, val, instance_settings):
        if val == '':
            val = rospy.get_param('~' + name, '')
        # load from instance_settings only if args/params aren't set
        if val == '' and instance_settings.contains(name):
            val = instance_settings.value(name)
        self.label[label_name].setText(val)
        return val

    def restore_settings(self, plugin_settings, instance_settings):
        self.source_frame = self.get_param_setting("source_frame", "source",
                                                   self.source_frame, instance_settings)
        self.target_frame = self.get_param_setting("target_frame", "target",
                                                   self.target_frame, instance_settings)

        for key in self.label.keys():
            name = key + '_hidden'
            if instance_settings.contains(name):
                if instance_settings.value(name) == str(True):
                    self.label[key].hide()
        self.setup_menu()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('source_frame', self.source_frame)
        instance_settings.set_value('target_frame', self.target_frame)

        for key in self.label.keys():
            is_hidden = self.label[key].isHidden()
            name = key + '_hidden'
            instance_settings.set_value(name, str(is_hidden))

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
