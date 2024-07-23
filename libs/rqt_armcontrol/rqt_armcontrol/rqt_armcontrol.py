'''
 *  rqt_armcontrol.py
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.

'''
import contextlib
import io
import os
import sys
from datetime import datetime
import csv

from ament_index_python import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog, QWidget

# pylint: enable=no-name-in-module,import-error

from rqt_gui.main import Main

from rqt_gui_py.plugin import Plugin

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from std_srvs.srv import SetBool


import rclpy
from rclpy.node import Node


class RqtCartesianController(Plugin):
    """rqt GUI plugin to visualize dot graphs."""

    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self._context = context
        self.subscription = None

        # only declare the parameter if running standalone or it's the first instance
        if self._context.serial_number() <= 1:
            self._context.node.declare_parameter("title", "Cartesian Controller")
        self.title = self._context.node.get_parameter("title").value

        self._widget = QWidget()
        self.setObjectName(self.title)

        _, self.package_path = get_resource("packages", "rqt_armcontrol")
        ui_file = os.path.join(
            self.package_path, "share", "rqt_armcontrol", "resource", "rqt_armcontrol.ui"
        )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName(self.title + "UI")

        title = self.title
        if self._context.serial_number() > 1:
            title += f" ({self._context.serial_number()})"
        self._widget.setWindowTitle(title)

        # only set main window title if running standalone
        if self._context.serial_number() < 1:
            self._widget.window().setWindowTitle(self.title)


        self.scale = 100.0 # Slider values between -scale and +scale

        self.cartesian_vel = TwistStamped()
        self.cartesian_vel.twist.linear.x = 0.0
        self.cartesian_vel.twist.linear.y = 0.0
        self.cartesian_vel.twist.linear.z = 0.0
        self.cartesian_vel.twist.angular.x = 0.0
        self.cartesian_vel.twist.angular.y = 0.0
        self.cartesian_vel.twist.angular.z = 0.0

        self.slider_released = True
        self.prev_slider_released = True

        context.add_widget(self._widget)
        self.setUpEventHandlers()

        self.publisher_ = self._context.node.create_publisher(TwistStamped, "/ros2_control_explorer/input_device_velocity", 1)
        timer_period = 0.02  # [sec] UI publishing rate
        self.timer = self._context.node.create_timer(timer_period, self.publisher_callback)

        self.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._context.node.get_logger().info("RQT Init Finished")


    def publisher_callback(self):
        if (not self.slider_released) or (not self.prev_slider_released) :
            self.cartesian_vel.header.stamp = self._context.node.get_clock().now().to_msg();
            self.publisher_.publish(self.cartesian_vel)
        self.prev_slider_released = self.slider_released

    def pose_sub_callback(self, msg):
        self.pose[0] = msg.position.x
        self.pose[1] = msg.position.y
        self.pose[2] = msg.position.z
        self.pose[3] = msg.orientation.w
        self.pose[4] = msg.orientation.x
        self.pose[5] = msg.orientation.y
        self.pose[6] = msg.orientation.z

    def setUpEventHandlers(self):
        self._widget.pos_x.valueChanged.connect(self.onXMove)
        self._widget.pos_y.valueChanged.connect(self.onYMove)
        self._widget.pos_z.valueChanged.connect(self.onZMove)
        self._widget.or_roll.valueChanged.connect(self.onRoMove)
        self._widget.or_pitch.valueChanged.connect(self.onPiMove)
        self._widget.or_yaw.valueChanged.connect(self.onYaMove)

        self._widget.pos_x.sliderReleased.connect(self.onSliderReleased)
        self._widget.pos_y.sliderReleased.connect(self.onSliderReleased)
        self._widget.pos_z.sliderReleased.connect(self.onSliderReleased)
        self._widget.or_roll.sliderReleased.connect(self.onSliderReleased)
        self._widget.or_pitch.sliderReleased.connect(self.onSliderReleased)
        self._widget.or_yaw.sliderReleased.connect(self.onSliderReleased)


    def onXMove(self, value):
        self.cartesian_vel.twist.linear.x = float(value) / self.scale
        self.slider_released = False

    def onYMove(self, value):
        self.cartesian_vel.twist.linear.y  = float(value) / self.scale
        self.slider_released = False

    def onZMove(self, value):
        self.cartesian_vel.twist.linear.z = float(value) / self.scale
        self.slider_released = False

    def onRoMove(self, value):
        self.cartesian_vel.twist.angular.x = float(value) / self.scale
        self.slider_released = False

    def onPiMove(self, value):
        self.cartesian_vel.twist.angular.y = float(value) / self.scale
        self.slider_released = False

    def onYaMove(self, value):
        self.cartesian_vel.twist.angular.z = float(value) / self.scale
        self.slider_released = False

    def onSliderReleased(self):
        self._widget.pos_x.setSliderPosition(0)
        self._widget.pos_y.setSliderPosition(0)
        self._widget.pos_z.setSliderPosition(0)
        self._widget.or_roll.setSliderPosition(0)
        self._widget.or_pitch.setSliderPosition(0)
        self._widget.or_yaw.setSliderPosition(0)
        self.cartesian_vel.twist.linear.x = 0.0
        self.cartesian_vel.twist.linear.y = 0.0
        self.cartesian_vel.twist.linear.z = 0.0
        self.cartesian_vel.twist.angular.x = 0.0
        self.cartesian_vel.twist.angular.y = 0.0
        self.cartesian_vel.twist.angular.z = 0.0
        self.slider_released = True


    # Qt methods
    def shutdown_plugin(self):
        """Shutdown plugin."""

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings."""

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore settings."""


def main():
    """Run the plugin."""
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_armcontrol")


if __name__ == "__main__":
    main()
