'''
 *  rqt_jointcontrol.py
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
            self.package_path, "share", "explorer_user_interfaces", "resource", "rqt_jointcontrol.ui"
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

        self.joint_vel = Float64MultiArray()
        self.joint_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

        #joint states
        self.joint = JointState()
        self.joint.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.joint.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_ = self._context.node.create_publisher(Float64MultiArray, "/explorer_user_interfaces/rqt_jointcontrol/dq_output", 1)
        timer_period = 0.02  # [sec] UI publishing rate
        self.timer = self._context.node.create_timer(timer_period, self.publisher_callback)

        self.joint_sub_ = self._context.node.create_subscription(JointState, '/joint_states', self.joint_sub_callback, 1)

        self._context.node.get_logger().info("RQT Init Finished")


    def publisher_callback(self):
        if (not self.slider_released) or (not self.prev_slider_released) :
            self.publisher_.publish(self.joint_vel)
        self.prev_slider_released = self.slider_released

    def setUpEventHandlers(self):
        self._widget.J1.valueChanged.connect(self.OnJ1Move)
        self._widget.J2.valueChanged.connect(self.OnJ2Move)
        self._widget.J3.valueChanged.connect(self.OnJ3Move)
        self._widget.J4.valueChanged.connect(self.OnJ4Move)
        self._widget.J5.valueChanged.connect(self.OnJ5Move)
        self._widget.J6.valueChanged.connect(self.OnJ6Move)
        self._widget.gripper.valueChanged.connect(self.onGripperMove)

        self._widget.J1.sliderReleased.connect(self.onSliderReleased)
        self._widget.J2.sliderReleased.connect(self.onSliderReleased)
        self._widget.J3.sliderReleased.connect(self.onSliderReleased)
        self._widget.J4.sliderReleased.connect(self.onSliderReleased)
        self._widget.J5.sliderReleased.connect(self.onSliderReleased)
        self._widget.J6.sliderReleased.connect(self.onSliderReleased)
        self._widget.gripper.sliderReleased.connect(self.onSliderReleased)


    def OnJ1Move(self, value):
        self.joint_vel.data[0] = float(value) / self.scale
        self.slider_released = False

    def OnJ2Move(self, value):
        self.joint_vel.data[1]  = float(value) / self.scale
        self.slider_released = False

    def OnJ3Move(self, value):
        self.joint_vel.data[2] = float(value) / self.scale
        self.slider_released = False

    def OnJ4Move(self, value):
        self.joint_vel.data[3] = float(value) / self.scale
        self.slider_released = False

    def OnJ5Move(self, value):
        self.joint_vel.data[4] = float(value) / self.scale
        self.slider_released = False

    def OnJ6Move(self, value):
        self.joint_vel.data[5] = float(value) / self.scale
        self.slider_released = False

    def onGripperMove(self, value):
        self.joint_vel.data[6] = float(value) / self.scale
        self.slider_released = False
    
    def onSliderReleased(self):
        self._widget.J1.setSliderPosition(0)
        self._widget.J2.setSliderPosition(0)
        self._widget.J3.setSliderPosition(0)
        self._widget.J4.setSliderPosition(0)
        self._widget.J5.setSliderPosition(0)
        self._widget.J6.setSliderPosition(0)
        self._widget.gripper.setSliderPosition(0)
        self.joint_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.slider_released = True

    def joint_sub_callback(self, msg):
        for i in range(0,6):
            j = 0
            while (self.joint.name[i]!= msg.name[j] and j<len(msg.name)):
                j += 1
            if(self.joint.name[i] == msg.name[j]):    
                self.joint.position[i] = msg.position[j]

        
        self._widget.J1_pos.setText("{:.2f} °".format(self.joint.position[0]*(180/3.141592)))
        self._widget.J2_pos.setText("{:.2f} °".format(self.joint.position[1]*(180/3.141592)))
        self._widget.J3_pos.setText("{:.2f} °".format(self.joint.position[2]*(180/3.141592)))
        self._widget.J4_pos.setText("{:.2f} °".format(self.joint.position[3]*(180/3.141592)))
        self._widget.J5_pos.setText("{:.2f} °".format(self.joint.position[4]*(180/3.141592)))
        self._widget.J6_pos.setText("{:.2f} °".format(self.joint.position[5]*(180/3.141592)))

    # Qt methods
    def shutdown_plugin(self):
        """Shutdown plugin."""

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings."""

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore settings."""


def main():
    """Run the plugin."""
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_jointcontrol")


if __name__ == "__main__":
    main()
