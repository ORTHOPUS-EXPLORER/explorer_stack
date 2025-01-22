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
from std_msgs.msg import Float64



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


        self.scale = 1000.0 # Slider values between -scale and +scale

        self.joint_vel = Float64()
        self.joint_vel.data = 0.0

        self.slider_released = True
        self.prev_slider_released = True

        context.add_widget(self._widget)
        self.setUpEventHandlers()

        self.publisher_ = self._context.node.create_publisher(Float64, "/ros2_control_actuator/dq_output", 1)
        timer_period = 0.02  # [sec] UI publishing rate
        self.timer = self._context.node.create_timer(timer_period, self.publisher_callback)


        self._context.node.get_logger().info("RQT Init Finished")


    def publisher_callback(self):
        if (not self.slider_released) or (not self.prev_slider_released) :
            self.publisher_.publish(self.joint_vel)
        self.prev_slider_released = self.slider_released

    def setUpEventHandlers(self):
        
        self._widget.actuator.valueChanged.connect(self.onActuatorMove)

        self._widget.actuator.sliderReleased.connect(self.onSliderReleased)

    def onActuatorMove(self, value):
        self.joint_vel.data = float(value) / self.scale
        self.slider_released = False
    
    def onSliderReleased(self):
        self._widget.actuator.setSliderPosition(0)
        self.joint_vel.data = 0.0
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
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_actuatorcontrol")


if __name__ == "__main__":
    main()
