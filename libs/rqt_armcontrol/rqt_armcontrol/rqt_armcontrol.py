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
        self.linear_speed.data = 0.05

        self.angular_speed= Float64()
        self.angular_speed.data = 1.0

        self.slider_released = True
        self.prev_slider_released = True

        self.select = Int64()
        self.select.data = 0

        self.zero_released = Bool()
        self.zero_released.data = False

        self.zero_pressed = Bool()
        self.zero_pressed.data = False

        self.J1_zero_released = Bool()
        self.J1_zero_released.data = False

        self.J1_zero_pressed = Bool()
        self.J1_zero_pressed.data = False

        self.J2_zero_released = Bool()
        self.J2_zero_released.data = False

        self.J2_zero_pressed = Bool()
        self.J2_zero_pressed.data = False

        self.J3_zero_released = Bool()
        self.J3_zero_released.data = False

        self.J3_zero_pressed = Bool()
        self.J3_zero_pressed.data = False

        self.J4_zero_released = Bool()
        self.J4_zero_released.data = False

        self.J4_zero_pressed = Bool()
        self.J4_zero_pressed.data = False

        self.J5_zero_released = Bool()
        self.J5_zero_released.data = False

        self.J5_zero_pressed = Bool()
        self.J5_zero_pressed.data = False

        self.J6_zero_released = Bool()
        self.J6_zero_released.data = False

        self.J6_zero_pressed = Bool()
        self.J6_zero_pressed.data = False

        self.home_released = Bool()
        self.home_released.data = False

        self.home_pressed = Bool()
        self.home_pressed.data = False

        # Add state tracking for home button operations
        self.home_button_is_pressed = False
        self.zero_button_is_pressed = False
        self.J1_zero_button_is_pressed = False
        self.J2_zero_button_is_pressed = False
        self.J3_zero_button_is_pressed = False
        self.J4_zero_button_is_pressed = False
        self.J5_zero_button_is_pressed = False
        self.J6_zero_button_is_pressed = False

        context.add_widget(self._widget)
        self.setUpEventHandlers()

        # Set fullscreen mode when running standalone (after widget is added to context)
        if self._context.serial_number() < 1:
            # Use a timer to delay maximizing to ensure widget is fully rendered
            from python_qt_binding.QtCore import QTimer
            QTimer.singleShot(1000, lambda: self._widget.window().showMaximized())

        self._widget.J1_pos.setText("{:.2f} °".format(0.00))
        self._widget.J2_pos.setText("{:.2f} °".format(0.00))
        self._widget.J3_pos.setText("{:.2f} °".format(0.00))
        self._widget.J4_pos.setText("{:.2f} °".format(0.00))
        self._widget.J5_pos.setText("{:.2f} °".format(0.00))
        self._widget.J6_pos.setText("{:.2f} °".format(0.00))

        # Set initial slider positions to match the default values
        self._widget.max_linear_speed.setSliderPosition(int(self.linear_speed.data * self.scale))
        self._widget.max_angular_speed.setSliderPosition(int(self.angular_speed.data * self.scale))

        #joint states
        self.joint = JointState()
        self.joint.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.joint.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_ = self._context.node.create_publisher(TwistStamped, "/ros2_control_explorer/input_device_velocity", 1)
        self.publisher_gripper_ = self._context.node.create_publisher(Float64, "/ros2_control_explorer/input_gripper_velocity", 1)
        self.publisher_linear_speed_ = self._context.node.create_publisher(Float64, "/ros2_control_explorer/max_linear_speed", 1)
        self.publisher_angular_speed_ = self._context.node.create_publisher(Float64, "/ros2_control_explorer/max_angular_speed", 1)
        self.publisher_spacemouse_select_ = self._context.node.create_publisher(Int64, "/ros2_control_explorer/spacemouse_select", 1)
        self.publisher_zero_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/zero_pressed", 1)
        self.publisher_zero_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/zero_released", 1)
        self.publisher_J1_zero_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J1_zero_pressed", 1)
        self.publisher_J1_zero_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J1_zero_released", 1)
        self.publisher_J2_zero_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J2_zero_pressed", 1)
        self.publisher_J2_zero_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J2_zero_released", 1)
        self.publisher_J3_zero_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J3_zero_pressed", 1)
        self.publisher_J3_zero_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J3_zero_released", 1)
        self.publisher_J4_zero_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J4_zero_pressed", 1)
        self.publisher_J4_zero_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J4_zero_released", 1)
        self.publisher_J5_zero_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J5_zero_pressed", 1)
        self.publisher_J5_zero_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J5_zero_released", 1)
        self.publisher_J6_zero_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J6_zero_pressed", 1)
        self.publisher_J6_zero_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/J6_zero_released", 1)
        self.publisher_home_pressed_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/home_pressed", 1)
        self.publisher_home_released_ = self._context.node.create_publisher(Bool, "/ros2_control_explorer/home_released", 1)
        timer_period = 0.02  # [sec] UI publishing rate
        self.timer = self._context.node.create_timer(timer_period, self.publisher_callback)

        self.joint_sub_ = self._context.node.create_subscription(JointState, '/joint_states', self.joint_sub_callback, 1)

        self._context.node.get_logger().info("RQT Init Finished")


    def publisher_callback(self):
        try:
            if (not self.slider_released) or (not self.prev_slider_released):
                self.cartesian_vel.header.stamp = self._context.node.get_clock().now().to_msg()
                self.publisher_.publish(self.cartesian_vel)
                self.publisher_gripper_.publish(self.gripper_pos)
            self.prev_slider_released = self.slider_released
            self.publisher_linear_speed_.publish(self.linear_speed)
            self.publisher_angular_speed_.publish(self.angular_speed)
            self.publisher_spacemouse_select_.publish(self.select)
            
            # Always publish current home_pressed state (true while held, false when released)
            self.publisher_home_pressed_.publish(self.home_pressed)

            self.publisher_zero_pressed_.publish(self.zero_pressed)

            self.publisher_J1_zero_pressed_.publish(self.J1_zero_pressed)
            self.publisher_J2_zero_pressed_.publish(self.J2_zero_pressed)
            self.publisher_J3_zero_pressed_.publish(self.J3_zero_pressed)
            self.publisher_J4_zero_pressed_.publish(self.J4_zero_pressed)
            self.publisher_J5_zero_pressed_.publish(self.J5_zero_pressed)
            self.publisher_J6_zero_pressed_.publish(self.J6_zero_pressed)

            
            # Only publish home_released signal once after button is released
            if self.home_released.data:
                self.publisher_home_released_.publish(self.home_released)
                self.home_released.data = False  # Reset after publishing once

            if self.zero_released.data:
                self.publisher_zero_released_.publish(self.zero_released)
                self.zero_released.data = False  # Reset after publishing once

            if self.J1_zero_released.data:
                self.publisher_J1_zero_released_.publish(self.J1_zero_released)
                self.J1_zero_released.data = False
                
            if self.J2_zero_released.data:
                self.publisher_J2_zero_released_.publish(self.J2_zero_released)
                self.J2_zero_released.data = False
            
            if self.J3_zero_released.data:
                self.publisher_J3_zero_released_.publish(self.J3_zero_released)
                self.J3_zero_released.data = False

            if self.J4_zero_released.data:
                self.publisher_J4_zero_released_.publish(self.J4_zero_released)
                self.J4_zero_released.data = False

            if self.J5_zero_released.data:
                self.publisher_J5_zero_released_.publish(self.J5_zero_released)
                self.J5_zero_released.data = False
            
            if self.J6_zero_released.data:
                self.publisher_J6_zero_released_.publish(self.J6_zero_released)
                self.J6_zero_released.data = False    

        except Exception as e:
            # Log error but don't crash the application
            self._context.node.get_logger().warn(f"Error in publisher_callback: {str(e)}")
        

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

        self._widget.zero_btn.released.connect(self.OnZeroReleased)
        self._widget.zero_btn.pressed.connect(self.OnZeroPressed)

        self._widget.J1_zero_btn.released.connect(self.OnJ1ZeroReleased)
        self._widget.J1_zero_btn.pressed.connect(self.OnJ1ZeroPressed)

        self._widget.J2_zero_btn.released.connect(self.OnJ2ZeroReleased)
        self._widget.J2_zero_btn.pressed.connect(self.OnJ2ZeroPressed)

        self._widget.J3_zero_btn.released.connect(self.OnJ3ZeroReleased)
        self._widget.J3_zero_btn.pressed.connect(self.OnJ3ZeroPressed)

        self._widget.J4_zero_btn.released.connect(self.OnJ4ZeroReleased)
        self._widget.J4_zero_btn.pressed.connect(self.OnJ4ZeroPressed)

        self._widget.J5_zero_btn.released.connect(self.OnJ5ZeroReleased)
        self._widget.J5_zero_btn.pressed.connect(self.OnJ5ZeroPressed)

        self._widget.J6_zero_btn.released.connect(self.OnJ6ZeroReleased)
        self._widget.J6_zero_btn.pressed.connect(self.OnJ6ZeroPressed)

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
        """Called when home button is released - stops homing process immediately."""
        if self.home_button_is_pressed:  # Only process if button was actually pressed
            self.home_button_is_pressed = False
            self.home_pressed.data = False
            self.home_released.data = True  # Signal to stop homing and resume normal control
            self._context.node.get_logger().info("Home button released - stopping homing process")

    def OnHomePressed(self):
        """Called when home button is pressed - starts continuous homing process."""
        if not self.home_button_is_pressed:  # Prevent multiple press events
            self.home_button_is_pressed = True
            self.home_pressed.data = True
            self.home_released.data = False
            self._context.node.get_logger().info("Home button pressed - starting homing process")
    
    def OnZeroReleased(self):
        """Called when zero button is released - stops zero process immediately."""
        if self.zero_button_is_pressed:  # Only process if button was actually pressed
            self.zero_button_is_pressed = False
            self.zero_pressed.data = False
            self.zero_released.data = True  # Signal to stop zero and resume normal control
            self._context.node.get_logger().info("Zero button released - stopping zero process")

    def OnZeroPressed(self):
        """Called when zero button is pressed - starts continuous zero process."""
        if not self.zero_button_is_pressed:  # Prevent multiple press events
            self.zero_button_is_pressed = True
            self.zero_pressed.data = True
            self.zero_released.data = False
            self._context.node.get_logger().info("Zero button pressed - starting zero process")

    def OnJ1ZeroReleased(self):
        """Called when J1 zero button is released - stops  J1 zero process immediately."""
        if self.J1_zero_button_is_pressed:  # Only process if button was actually pressed
            self.J1_zero_button_is_pressed = False
            self.J1_zero_pressed.data = False
            self.J1_zero_released.data = True  # Signal to stop zero and resume normal control
            self._context.node.get_logger().info("J1 zero button released - stopping J1 zero process")

    def OnJ1ZeroPressed(self):
        """Called when J1 zero button is pressed - starts continuous J1 zero process."""
        if not self.J1_zero_button_is_pressed:  # Prevent multiple press events
            self.J1_zero_button_is_pressed = True
            self.J1_zero_pressed.data = True
            self.J1_zero_released.data = False
            self._context.node.get_logger().info("J1 zero button pressed - starting J1 zero process")

    def OnJ2ZeroReleased(self):
        """Called when J2 zero button is released - stops  J2 zero process immediately."""
        if self.J2_zero_button_is_pressed:  # Only process if button was actually pressed
            self.J2_zero_button_is_pressed = False
            self.J2_zero_pressed.data = False
            self.J2_zero_released.data = True  # Signal to stop zero and resume normal control
            self._context.node.get_logger().info("J2 zero button released - stopping J2 zero process")

    def OnJ2ZeroPressed(self):
        """Called when J2 zero button is pressed - starts continuous J2 zero process."""
        if not self.J2_zero_button_is_pressed:  # Prevent multiple press events
            self.J2_zero_button_is_pressed = True
            self.J2_zero_pressed.data = True
            self.J2_zero_released.data = False
            self._context.node.get_logger().info("J2 zero button pressed - starting J2 zero process")

    def OnJ3ZeroReleased(self):
        """Called when J3 zero button is released - stops  J3 zero process immediately."""
        if self.J3_zero_button_is_pressed:  # Only process if button was actually pressed
            self.J3_zero_button_is_pressed = False
            self.J3_zero_pressed.data = False
            self.J3_zero_released.data = True  # Signal to stop zero and resume normal control
            self._context.node.get_logger().info("J3 zero button released - stopping J3 zero process")

    def OnJ3ZeroPressed(self):
        """Called when J3 zero button is pressed - starts continuous J3 zero process."""
        if not self.J3_zero_button_is_pressed:  # Prevent multiple press events
            self.J3_zero_button_is_pressed = True
            self.J3_zero_pressed.data = True
            self.J3_zero_released.data = False
            self._context.node.get_logger().info("J3 zero button pressed - starting J3 zero process")

    def OnJ4ZeroReleased(self):
        """Called when J4 zero button is released - stops  J4 zero process immediately."""
        if self.J4_zero_button_is_pressed:  # Only process if button was actually pressed
            self.J4_zero_button_is_pressed = False
            self.J4_zero_pressed.data = False
            self.J4_zero_released.data = True  # Signal to stop zero and resume normal control
            self._context.node.get_logger().info("J4 zero button released - stopping J4 zero process")

    def OnJ4ZeroPressed(self):
        """Called when J4 zero button is pressed - starts continuous J4 zero process."""
        if not self.J4_zero_button_is_pressed:  # Prevent multiple press events
            self.J4_zero_button_is_pressed = True
            self.J4_zero_pressed.data = True
            self.J4_zero_released.data = False
            self._context.node.get_logger().info("J4 zero button pressed - starting J4 zero process")

    def OnJ5ZeroReleased(self):
        """Called when J5 zero button is released - stops  J5 zero process immediately."""
        if self.J5_zero_button_is_pressed:  # Only process if button was actually pressed
            self.J5_zero_button_is_pressed = False
            self.J5_zero_pressed.data = False
            self.J5_zero_released.data = True  # Signal to stop zero and resume normal control
            self._context.node.get_logger().info("J5 zero button released - stopping J5 zero process")

    def OnJ5ZeroPressed(self):
        """Called when J5 zero button is pressed - starts continuous J5 zero process."""
        if not self.J5_zero_button_is_pressed:  # Prevent multiple press events
            self.J5_zero_button_is_pressed = True
            self.J5_zero_pressed.data = True
            self.J5_zero_released.data = False
            self._context.node.get_logger().info("J5 zero button pressed - starting J5 zero process")

    def OnJ6ZeroReleased(self):
        """Called when J6 zero button is released - stops  J6 zero process immediately."""
        if self.J6_zero_button_is_pressed:  # Only process if button was actually pressed
            self.J6_zero_button_is_pressed = False
            self.J6_zero_pressed.data = False
            self.J6_zero_released.data = True  # Signal to stop zero and resume normal control
            self._context.node.get_logger().info("J6 zero button released - stopping J6 zero process")

    def OnJ6ZeroPressed(self):
        """Called when J1 zero button is pressed - starts continuous J1 zero process."""
        if not self.J6_zero_button_is_pressed:  # Prevent multiple press events
            self.J6_zero_button_is_pressed = True
            self.J6_zero_pressed.data = True
            self.J6_zero_released.data = False
            self._context.node.get_logger().info("J6 zero button pressed - starting J6 zero process")

    def onSliderReleased(self):
        # Reset velocities immediately  
        self.cartesian_vel.twist.linear.x = 0.0
        self.cartesian_vel.twist.linear.y = 0.0
        self.cartesian_vel.twist.linear.z = 0.0
        self.cartesian_vel.twist.angular.x = 0.0
        self.cartesian_vel.twist.angular.y = 0.0
        self.cartesian_vel.twist.angular.z = 0.0
        self.gripper_pos.data = 0.0
        self.slider_released = True
        
        # Reset slider positions with a small delay to avoid UI conflicts
        from python_qt_binding.QtCore import QTimer
        QTimer.singleShot(10, self._reset_slider_positions)
    
    def _reset_slider_positions(self):
        """Reset slider positions to center (0) with proper signal blocking."""
        # Temporarily block signals to prevent triggering value change events
        sliders = [
            self._widget.pos_x,
            self._widget.pos_y, 
            self._widget.pos_z,
            self._widget.or_roll,
            self._widget.or_pitch,
            self._widget.or_yaw,
            self._widget.gripper
        ]
        
        for slider in sliders:
            try:
                slider.blockSignals(True)
                slider.setValue(0)  # Use setValue instead of setSliderPosition
                slider.setSliderPosition(0)
                slider.blockSignals(False)
                slider.update()  # Force UI update
            except Exception as e:
                # If one slider fails, continue with others
                self._context.node.get_logger().warn(f"Error resetting slider: {str(e)}")
                slider.blockSignals(False)  # Ensure signals are unblocked

    def joint_sub_callback(self, msg):
        try:
            # Ensure msg has required attributes
            if not hasattr(msg, 'name') or not hasattr(msg, 'position'):
                return
            
            for i in range(0, 6):
                # Safely search for the joint name in the message
                for j in range(len(msg.name)):
                    if self.joint.name[i] == msg.name[j]:
                        # Ensure we don't access position array out of bounds
                        if j < len(msg.position):
                            self.joint.position[i] = msg.position[j]
                        break

            # Update UI with joint positions (convert from radians to degrees)
            self._widget.J1_pos.setText("{:.2f} °".format(self.joint.position[0]*(180/3.141592)))
            self._widget.J2_pos.setText("{:.2f} °".format(self.joint.position[1]*(180/3.141592)))
            self._widget.J3_pos.setText("{:.2f} °".format(self.joint.position[2]*(180/3.141592)))
            self._widget.J4_pos.setText("{:.2f} °".format(self.joint.position[3]*(180/3.141592)))
            self._widget.J5_pos.setText("{:.2f} °".format(self.joint.position[4]*(180/3.141592)))
            self._widget.J6_pos.setText("{:.2f} °".format(self.joint.position[5]*(180/3.141592)))
        except Exception as e:
            # Log error but don't crash the application
            self._context.node.get_logger().warn(f"Error in joint_sub_callback: {str(e)}")
            


    # Qt methods
    def shutdown_plugin(self):
        """Shutdown plugin."""
        try:
            # Clean up timer and subscribers
            if hasattr(self, 'timer'):
                self.timer.cancel()
            if hasattr(self, 'joint_sub_'):
                self._context.node.destroy_subscription(self.joint_sub_)
        except Exception as e:
            self._context.node.get_logger().warn(f"Error during shutdown: {str(e)}")

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings."""

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore settings."""


def main():
    """Run the plugin."""
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_armcontrol")


if __name__ == "__main__":
    main()
