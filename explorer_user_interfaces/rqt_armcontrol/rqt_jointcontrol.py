"""
*  rqt_jointcontrol.py
*  Copyright (C) 2022 Orthopus
*  All rights reserved.

"""

import os
import sys
from functools import partial

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class RqtJointController(Plugin):
    """rqt GUI plugin to visualize dot graphs."""

    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self._context = context
        self.subscription = None

        # only declare the parameter if running standalone or it's the first instance
        if self._context.serial_number() <= 1:
            self._context.node.declare_parameter("title", "Joint Controller")
        self.title = self._context.node.get_parameter("title").value

        self._widget = QWidget()
        self.setObjectName(self.title)

        _, self.package_path = get_resource("packages", "explorer_user_interfaces")
        ui_file = os.path.join(
            self.package_path,
            "share",
            "explorer_user_interfaces",
            "resource",
            "rqt_jointcontrol.ui",
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

        self.scale = 1000.0  # Slider values between -scale and +scale

        self.joint_vel = Float64MultiArray()
        self.joint_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.gripper_vel = Float64MultiArray()
        self.gripper_vel.data = [0.0]

        self.slider_released = True
        self.prev_slider_released = True

        context.add_widget(self._widget)
        self.setUpEventHandlers()

        self._widget.J1_pos.setText("{:.2f} °".format(0.00))
        self._widget.J2_pos.setText("{:.2f} °".format(0.00))
        self._widget.J3_pos.setText("{:.2f} °".format(0.00))
        self._widget.J4_pos.setText("{:.2f} °".format(0.00))
        self._widget.J5_pos.setText("{:.2f} °".format(0.00))
        self._widget.J6_pos.setText("{:.2f} °".format(0.00))

        # joint states
        self.joint = JointState()
        self.joint.name = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        self.joint.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joints_publisher_ = self._context.node.create_publisher(
            Float64MultiArray, "/explorer_user_interfaces/rqt_jointcontrol/dq_output", 1
        )
        self.gripper_publisher_ = self._context.node.create_publisher(
            Float64MultiArray,
            "/explorer_user_interfaces/rqt_armcontrol/input_gripper_velocity",
            1,
        )
        timer_period = 0.02  # [sec] UI publishing rate
        self.timer = self._context.node.create_timer(
            timer_period, self.publisher_callback
        )

        ## TODO: joint state has 250hz and its frequency is way too high for Python GIL to handle => slider feels laggy even with empty callback
        ## Note: destroying subscription via node.destroy_subscription cause InvalidHandle like 1/10 times so there's no quickfix available for now
        ## Hint: Having a second throttled topic could solve the issue without any issue (topic_tools/topic_throttle)
        self.joint_sub_ = self._context.node.create_subscription(
            JointState, "/joint_states", self.joint_sub_callback, 2
        )
        self._context.node.get_logger().info("RQT Init Finished")

        self.position_ratio_ = 180 / 3.141592

    def publisher_callback(self):
        if (not self.slider_released) or (not self.prev_slider_released):
            self.joints_publisher_.publish(self.joint_vel)
            self.gripper_publisher_.publish(self.gripper_vel)
        self.prev_slider_released = self.slider_released

    def setUpEventHandlers(self):
        self._widget.J1.valueChanged.connect(partial(self.OnJointMove, 0))
        self._widget.J2.valueChanged.connect(partial(self.OnJointMove, 1))
        self._widget.J3.valueChanged.connect(partial(self.OnJointMove, 2))
        self._widget.J4.valueChanged.connect(partial(self.OnJointMove, 3))
        self._widget.J5.valueChanged.connect(partial(self.OnJointMove, 4))
        self._widget.J6.valueChanged.connect(partial(self.OnJointMove, 5))
        self._widget.gripper.valueChanged.connect(self.onGripperMove)

        self._widget.J1.sliderReleased.connect(partial(self.onSliderReleased, "J1"))
        self._widget.J2.sliderReleased.connect(partial(self.onSliderReleased, "J2"))
        self._widget.J3.sliderReleased.connect(partial(self.onSliderReleased, "J3"))
        self._widget.J4.sliderReleased.connect(partial(self.onSliderReleased, "J4"))
        self._widget.J5.sliderReleased.connect(partial(self.onSliderReleased, "J5"))
        self._widget.J6.sliderReleased.connect(partial(self.onSliderReleased, "J6"))
        self._widget.gripper.sliderReleased.connect(
            partial(self.onSliderReleased, "gripper")
        )

    def OnJointMove(self, index, value):
        self.joint_vel.data[index] = float(value) / self.scale
        self.slider_released = False

    def onGripperMove(self, value):
        self.gripper_vel.data[0] = float(value) / self.scale
        self.slider_released = False

    def onSliderReleased(self, widget_name: str):
        target_widget = getattr(self._widget, widget_name)
        target_widget.setSliderPosition(0)

        self.joint_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_vel.data = [0.0]
        self.slider_released = True

    def joint_sub_callback(self, msg):
        for i in range(0, 6):
            target_widget = getattr(self._widget, "J" + str(i + 1) + "_pos")
            j = 0
            while self.joint.name[i] != msg.name[j] and j < len(msg.name):
                j += 1
            if self.joint.name[i] == msg.name[j]:
                self.joint.position[i] = msg.position[j]
                self.modify_widget_text(target_widget, msg.position[j])

    def modify_widget_text(self, widget_target, position: float):
        widget_target.setText("{:.2f} °".format(position * self.position_ratio_))

    # Qt methods
    def shutdown_plugin(self):
        """Shutdown plugin."""
        self._context.node.destroy_subscription(self.joint_sub_)

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings."""

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore settings."""


def main():
    """Run the plugin."""
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_jointcontrol")


if __name__ == "__main__":
    main()
