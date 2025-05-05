'''
 *  rqt_actuatorcontrol.py
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.

'''
import os
import sys

import yaml

from ament_index_python import get_resource


from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

# pylint: enable=no-name-in-module,import-error

from rqt_gui.main import Main

from rqt_gui_py.plugin import Plugin

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray



class RqtActuatorController(Plugin):
    """rqt GUI plugin to visualize dot graphs."""

    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self._context = context
        self.subscription = None

        # only declare the parameter if running standalone or it's the first instance
        if self._context.serial_number() <= 1:
            self._context.node.declare_parameter("title", "Actuator Controller")
        self.title = self._context.node.get_parameter("title").value

        self._widget = QWidget()
        self.setObjectName(self.title)

        _, self.package_path = get_resource("packages", "rqt_armcontrol")
        ui_file = os.path.join(
            self.package_path, "share", "rqt_armcontrol", "resource", "rqt_actuatorcontrol.ui"
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

        self.velocity_scale = 1000.0 
        self.effort_scale = 30.0 

        self.joint_vel = Float64()
        self.joint_vel.data = 0.0

        self.joint_velocity = Float64MultiArray()
        self.joint_velocity.data = [0.0]

        self.joint_effort = Float64MultiArray()
        self.joint_effort.data = [0.0]

        context.add_widget(self._widget)
        self.setUpEventHandlers()

        self.publisher_vel_pos_ = self._context.node.create_publisher(Float64, "/ros2_control_actuator/dq_output", 1)
        self.publisher_velocity_ = self._context.node.create_publisher(Float64, "/forward_velocity_controller/commands", 1)
        self.publisher_torque_ = self._context.node.create_publisher(Float64MultiArray, "/forward_effort_controller/commands", 1)
        timer_period = 0.02  # [sec] UI publishing rate
        self.timer = self._context.node.create_timer(timer_period, self.publisher_callback)


        self._context.node.get_logger().info("RQT Init Finished")


    def publisher_callback(self):
        self.publisher_vel_pos_.publish(self.joint_vel)
        self.publisher_velocity_.publish(self.joint_velocity)
        self.publisher_torque_.publish(self.joint_effort)

    def setUpEventHandlers(self):
        
        self._widget.velocity.valueChanged.connect(self.onVelocityMove)

        self._widget.torque.valueChanged.connect(self.onTorqueMove)

        self._widget.zero_btn.pressed.connect(self.OnZeroPressed)

    def onVelocityMove(self, value):
        self.joint_vel.data = float(value) / self.velocity_scale
        self.joint_velocity.data[0] = float(value) / self.velocity_scale

    def onTorqueMove(self, value):
        self.joint_effort.data[0] = float(value) / self.effort_scale

    def OnZeroPressed(self):
        self._widget.velocity.setSliderPosition(0)
        self.joint_vel.data = 0.0
        self.joint_velocity.data[0] = 0.0
        self._widget.torque.setSliderPosition(0)
        self.joint_effort.data[0] = 0.0

    # Qt methods
    def shutdown_plugin(self):
        """Shutdown plugin."""

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings."""

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore settings."""


def main():
    """Run the plugin."""
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_actuatorcontrol")


if __name__ == "__main__":
    main()
