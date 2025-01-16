'''
 *  rqt_armcontrol.py
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.

'''
import os
import sys


from ament_index_python import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

# pylint: enable=no-name-in-module,import-error

from rqt_gui.main import Main

from rqt_gui_py.plugin import Plugin

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState



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


        self.scale = 1000.0 # Slider values between -scale and +scale

        self.cartesian_vel = TwistStamped()
        self.cartesian_vel.twist.linear.x = 0.0
        self.cartesian_vel.twist.linear.y = 0.0
        self.cartesian_vel.twist.linear.z = 0.0
        self.cartesian_vel.twist.angular.x = 0.0
        self.cartesian_vel.twist.angular.y = 0.0
        self.cartesian_vel.twist.angular.z = 0.0

        self.gripper_pos= Float64()
        self.gripper_pos.data = 0.0
    
        self.linear_speed= Float64()
        self.linear_speed.data = 0.025

        self.angular_speed= Float64()
        self.angular_speed.data = 0.5

        self.slider_released = True
        self.prev_slider_released = True

        self.select = Int64()
        self.select.data = 0

        self.home_released = Bool()
        self.home_released.data = False

        self.home_pressed = Bool()
        self.home_pressed.data = False

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

        self.publisher_ = self._context.node.create_publisher(TwistStamped, "/ros2_control_explorer/input_device_velocity", 1)
        self.publisher_gripper_ = self._context.node.create_publisher(Float64, "/ros2_control_explorer/input_gripper_velocity", 1)
        self.publisher_linear_speed_ = self._context.node.create_publisher(Float64, "/ros2_control_explorer/max_linear_speed", 1)
        self.publisher_angular_speed_ = self._context.node.create_publisher(Float64, "/ros2_control_explorer/max_angular_speed", 1)
        self.publisher_spacemouse_select_ = self._context.node.create_publisher(Int64, "/ros2_control_explorer/spacemouse_select", 1)
        self.publisher_home_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/home_pressed", 1)
        self.publisher_home_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/home_released", 1)
        timer_period = 0.02  # [sec] UI publishing rate
        self.timer = self._context.node.create_timer(timer_period, self.publisher_callback)

        self.joint_sub_ = self._context.node.create_subscription(JointState, '/joint_states', self.joint_sub_callback, 1)

        self._context.node.get_logger().info("RQT Init Finished")


    def publisher_callback(self):
        if (not self.slider_released) or (not self.prev_slider_released) :
            self.cartesian_vel.header.stamp = self._context.node.get_clock().now().to_msg();
            self.publisher_.publish(self.cartesian_vel)
            self.publisher_gripper_.publish(self.gripper_pos)
        self.prev_slider_released = self.slider_released
        self.publisher_linear_speed_.publish(self.linear_speed)
        self.publisher_angular_speed_.publish(self.angular_speed)
        self.publisher_spacemouse_select_.publish(self.select)
        if(self.home_released.data == True ):
            self.home_released.data = False
            self.publisher_home_released_.publish(self.home_released)
        

    def setUpEventHandlers(self):
        self._widget.pos_x.valueChanged.connect(self.onXMove)
        self._widget.pos_y.valueChanged.connect(self.onYMove)
        self._widget.pos_z.valueChanged.connect(self.onZMove)
        self._widget.or_roll.valueChanged.connect(self.onRoMove)
        self._widget.or_pitch.valueChanged.connect(self.onPiMove)
        self._widget.or_yaw.valueChanged.connect(self.onYaMove)

        self._widget.gripper.valueChanged.connect(self.onGripperMove)

        self._widget.max_linear_speed.valueChanged.connect(self.onLinearSpeedMove)
        self._widget.max_angular_speed.valueChanged.connect(self.onAngularSpeedMove)

        self._widget.spacemouse_select.valueChanged.connect(self.OnChangeSelect)

        self._widget.home_btn.released.connect(self.OnHomeReleased)
        self._widget.home_btn.pressed.connect(self.OnHomePressed)

        self._widget.pos_x.sliderReleased.connect(self.onSliderReleased)
        self._widget.pos_y.sliderReleased.connect(self.onSliderReleased)
        self._widget.pos_z.sliderReleased.connect(self.onSliderReleased)
        self._widget.or_roll.sliderReleased.connect(self.onSliderReleased)
        self._widget.or_pitch.sliderReleased.connect(self.onSliderReleased)
        self._widget.or_yaw.sliderReleased.connect(self.onSliderReleased)
        self._widget.gripper.sliderReleased.connect(self.onSliderReleased)


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

    def onGripperMove(self, value):
        self.gripper_pos.data = float(value) / self.scale
        self.slider_released = False
    
    def onLinearSpeedMove(self, value):
        self.linear_speed.data = float(value) / self.scale
    
    def onAngularSpeedMove(self, value):
        self.angular_speed.data = float(value) / self.scale

    def OnChangeSelect(self, value):
        self.select.data = value

    def OnHomeReleased(self):
        self.home_released.data = True
        self.home_pressed.data = False
        self.publisher_home_released_.publish(self.home_released)
        self.publisher_home_pressed_.publish(self.home_pressed)

    def OnHomePressed(self):
        self.home_pressed.data = True
        self.publisher_home_pressed_.publish(self.home_pressed)

    def onSliderReleased(self):
        self._widget.pos_x.setSliderPosition(0)
        self._widget.pos_y.setSliderPosition(0)
        self._widget.pos_z.setSliderPosition(0)
        self._widget.or_roll.setSliderPosition(0)
        self._widget.or_pitch.setSliderPosition(0)
        self._widget.or_yaw.setSliderPosition(0)
        self._widget.gripper.setSliderPosition(0)
        self.cartesian_vel.twist.linear.x = 0.0
        self.cartesian_vel.twist.linear.y = 0.0
        self.cartesian_vel.twist.linear.z = 0.0
        self.cartesian_vel.twist.angular.x = 0.0
        self.cartesian_vel.twist.angular.y = 0.0
        self.cartesian_vel.twist.angular.z = 0.0
        self.gripper_pos.data = 0.0
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
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_armcontrol")


if __name__ == "__main__":
    main()
