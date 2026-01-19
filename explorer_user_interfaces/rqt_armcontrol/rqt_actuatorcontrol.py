import os
import sys
import math

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin

from std_msgs.msg import Float64, Float64MultiArray

# Service for mode switching
from orthopus_vesc_interfaces.srv import SetMode

class RqtActuatorController(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._context = context

        # Parameters
        if self._context.serial_number() <= 1:
            self._context.node.declare_parameter("title", "Actuator Controller")
            self._context.node.declare_parameter("joint_name", "joint")
            self._context.node.declare_parameter("mode_service", "/actuator/mode")

        self.title = self._context.node.get_parameter("title").value
        self.joint_name = self._context.node.get_parameter("joint_name").value
        self.mode_service_name = self._context.node.get_parameter("mode_service").value

        self._widget = QWidget()
        self.setObjectName(self.title)

        _, self.package_path = get_resource("packages", "explorer_user_interfaces")
        ui_file = os.path.join(
            self.package_path, "share", "explorer_user_interfaces", "resource", "rqt_actuatorcontrol.ui"
        )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName(self.title + "UI")

        title = self.title
        if self._context.serial_number() > 1:
            title += f" ({self._context.serial_number()})"
        self._widget.setWindowTitle(title)

        if self._context.serial_number() < 1:
            self._widget.window().setWindowTitle(self.title)

        # UI option: position command generated from velocity (via external integrator node)
        self.pos_from_vel_enabled = False

        # Internal state
        self._last_mode_sent = None
        self._last_joint_index = None


        # Slider scaling
        vel_slider_max = 1000
        vel_max_rad_s = 5.0
        self.vel_scale = vel_max_rad_s / vel_slider_max

        trq_slider_max = 1000
        trq_max_nm = 10.0
        self.trq_scale = trq_max_nm / trq_slider_max

        # ----------------------------
        # Messages (commands)
        # ----------------------------
        self.joint_vel = Float64(data=0.0)                # rad/s -> external integrator node
        self.joint_position = Float64MultiArray(data=[0.0])  # rad
        self.joint_velocity = Float64MultiArray(data=[0.0])  # rad/s
        self.joint_effort = Float64MultiArray(data=[0.0])    # N.m

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        # publishers
        self.publisher_vel_pos_ = self._context.node.create_publisher(
            Float64, "/ros2_control_actuator/dq_output", 1
        )
        self.publisher_position_ = self._context.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 1
        )
        self.publisher_velocity_ = self._context.node.create_publisher(
            Float64MultiArray, "/forward_velocity_controller/commands", 1
        )
        self.publisher_torque_ = self._context.node.create_publisher(
            Float64MultiArray, "/forward_effort_controller/commands", 1
        )


        # Mode service client
        self.mode_client = self._context.node.create_client(SetMode, self.mode_service_name)

        # UI wiring
        context.add_widget(self._widget)
        self.setUpEventHandlers()
        self._sync_all_value_labels()
        self.update_ui_enabled_state()

        # Timer (publish loop)
        self.dt = 0.02
        self.timer = self._context.node.create_timer(self.dt, self.publisher_callback)

        self._context.node.get_logger().info("RQT Init Finished")

    # ----------------------------
    # Mode selection
    # ----------------------------
    def _get_selected_mode(self) -> str:
        if self._widget.mode_off.isChecked():
            return "OFF"
        if self._widget.mode_position.isChecked():
            return "Position"
        if self._widget.mode_velocity.isChecked():
            return "Velocity"
        if self._widget.mode_effort.isChecked():
            return "Effort"
        if self._widget.mode_impedance.isChecked():
            return "Impedance"
        return "OFF"

    def _mode_to_service_string(self, mode: str) -> str:
        # Map UI label -> service mode string expected by your controller
        # Based on your bash example: "impedance"
        m = mode.lower()
        if m == "off":
            return "off"
        if m == "position":
            return "position"
        if m == "velocity":
            return "velocity"
        if m == "effort":
            return "effort"
        if m == "impedance":
            return "impedance"
        return "off"

    def _maybe_call_mode_service(self, mode: str):
        service_mode = self._mode_to_service_string(mode)
        if service_mode == self._last_mode_sent:
            return  # no change -> no call

        self._last_mode_sent = service_mode

        if not self.mode_client.service_is_ready():
            # don't block the UI; just warn once per change
            self._context.node.get_logger().warn(
                f"Mode service '{self.mode_service_name}' not ready; cannot set mode '{service_mode}'."
            )
            return

        req = SetMode.Request()
        req.joint_name = self.joint_name
        req.mode = service_mode

        future = self.mode_client.call_async(req)

        def _done_cb(f):
            try:
                resp = f.result()
                # If the service has a success/ message field, you can log it here.
                self._context.node.get_logger().info(
                    f"Mode set request sent: joint='{self.joint_name}', mode='{service_mode}'"
                )
            except Exception as e:
                self._context.node.get_logger().error(f"Mode service call failed: {e}")

        future.add_done_callback(_done_cb)

    # ----------------------------
    # Publishing logic
    # ----------------------------
    def publisher_callback(self):
        mode = self._get_selected_mode()

        # Push mode to controller when it changes
        self._maybe_call_mode_service(mode)

        if mode == "OFF":
            self.publisher_vel_pos_.publish(Float64(data=0.0))
            z = Float64MultiArray(data=[0.0])
            self.publisher_position_.publish(z)
            self.publisher_velocity_.publish(z)
            self.publisher_torque_.publish(z)
            return

        if mode == "Position":
            # If enabled, publish velocity (rad/s) to external integrator node
            if self.pos_from_vel_enabled:
                self.publisher_vel_pos_.publish(self.joint_vel)
            else:
                self.publisher_position_.publish(self.joint_position)
            return

        if mode == "Velocity":
            self.publisher_velocity_.publish(self.joint_velocity)
            return

        if mode == "Effort":
            self.publisher_torque_.publish(self.joint_effort)
            return

        if mode == "Impedance":
            # Your chosen behavior:
            # - torque always
            # - velocity direct always
            # - and either external integrator vel OR direct position
            self.publisher_torque_.publish(self.joint_effort)
            self.publisher_velocity_.publish(self.joint_velocity)
            if self.pos_from_vel_enabled:
                self.publisher_vel_pos_.publish(self.joint_vel)
            else:
                self.publisher_position_.publish(self.joint_position)
            return

    # ----------------------------
    # UI wiring
    # ----------------------------
    def setUpEventHandlers(self):
        self._widget.position.valueChanged.connect(self.onPositionMove)
        self._widget.velocity.valueChanged.connect(self.onVelocityMove)
        self._widget.torque.valueChanged.connect(self.onTorqueMove)

        self._widget.pos_zero_btn.pressed.connect(self.onPosZeroPressed)
        self._widget.vel_zero_btn.pressed.connect(self.onVelZeroPressed)
        self._widget.trq_zero_btn.pressed.connect(self.onTrqZeroPressed)

        # Checkbox (must exist in .ui as name="pos_from_vel_cb")
        if hasattr(self._widget, "pos_from_vel_cb"):
            self._widget.pos_from_vel_cb.toggled.connect(self.onPosFromVelToggled)

        # Refresh UI enabled/disabled state when mode changes
        self._widget.mode_off.toggled.connect(self.update_ui_enabled_state)
        self._widget.mode_position.toggled.connect(self.update_ui_enabled_state)
        self._widget.mode_velocity.toggled.connect(self.update_ui_enabled_state)
        self._widget.mode_effort.toggled.connect(self.update_ui_enabled_state)
        self._widget.mode_impedance.toggled.connect(self.update_ui_enabled_state)

        # Initial state
        self.update_ui_enabled_state()

    def onPosFromVelToggled(self, checked: bool):
        self.pos_from_vel_enabled = checked
        self.update_ui_enabled_state()

    def update_ui_enabled_state(self, *_):
        mode = self._get_selected_mode()

        pos_en = False
        vel_en = False
        trq_en = False

        if mode == "OFF":
            pos_en = vel_en = trq_en = False

        elif mode == "Position":
            if self.pos_from_vel_enabled:
                pos_en = False
                vel_en = True
            else:
                pos_en = True
                vel_en = False
            trq_en = False

        elif mode == "Velocity":
            pos_en = False
            vel_en = True
            trq_en = False

        elif mode == "Effort":
            pos_en = False
            vel_en = False
            trq_en = True

        elif mode == "Impedance":
            trq_en = True
            vel_en = True
            pos_en = not self.pos_from_vel_enabled

        self._widget.position.setEnabled(pos_en)
        self._widget.velocity.setEnabled(vel_en)
        self._widget.torque.setEnabled(trq_en)

        self._widget.pos_zero_btn.setEnabled(pos_en)
        self._widget.vel_zero_btn.setEnabled(vel_en)
        self._widget.trq_zero_btn.setEnabled(trq_en)

        if hasattr(self._widget, "pos_from_vel_cb"):
            self._widget.pos_from_vel_cb.setEnabled(mode in ("Position", "Impedance"))

    # ----------------------------
    # Slider callbacks (commands)
    # ----------------------------
    def onPositionMove(self, value_deg: int):
        cmd_rad = float(value_deg) * math.pi / 180.0
        self.joint_position.data[0] = cmd_rad

        # Update legacy UI labels (if you keep them)
        if hasattr(self._widget, "pos_value"):
            self._widget.pos_value.setText(str(int(value_deg)))

        # Update new command labels (recommended)
        self._set_label("pos_cmd_deg", f"{int(value_deg)}")
        self._set_label("pos_cmd_rad", f"{cmd_rad:.4f}")
        self._widget.pos_deg_value.setText(f"{value_deg} deg")
        self._widget.pos_rad_value.setText(f"{cmd_rad:.4f} rad")


    def onVelocityMove(self, value: int):
        vel_rad_s = value * self.vel_scale

        self.joint_velocity.data[0] = vel_rad_s
        self.joint_vel.data = vel_rad_s  # vers intégrateur externe

        # UI
        deg_s = vel_rad_s * 180.0 / math.pi
        self._widget.vel_deg_s_value.setText(f"{deg_s:.1f} deg/s")
        self._widget.vel_rad_s_value.setText(f"{vel_rad_s:.3f} rad/s")

    def onTorqueMove(self, value: int):
        trq_nm = value * self.trq_scale
        self.joint_effort.data[0] = trq_nm

        self._widget.trq_nm_value.setText(f"{trq_nm:.2f} N.m")

    # ----------------------------
    # Zero buttons
    # ----------------------------
    def onPosZeroPressed(self):
        self._widget.position.setSliderPosition(0)
        self.joint_position.data[0] = 0.0
        if hasattr(self._widget, "pos_value"):
            self._widget.pos_value.setText("0")
        self._set_label("pos_cmd_deg", "0")
        self._set_label("pos_cmd_rad", "0.0000")

    def onVelZeroPressed(self):
        self._widget.velocity.setSliderPosition(0)
        self.joint_vel.data = 0.0
        self.joint_velocity.data[0] = 0.0
        if hasattr(self._widget, "vel_value"):
            self._widget.vel_value.setText("0")
        self._set_label("vel_cmd_deg_s", "0")
        self._set_label("vel_cmd_rad_s", "0.0000")

    def onTrqZeroPressed(self):
        self._widget.torque.setSliderPosition(0)
        self.joint_effort.data[0] = 0.0
        if hasattr(self._widget, "trq_value"):
            self._widget.trq_value.setText("0")
        self._set_label("trq_cmd_nm", "0.00")

    # ----------------------------
    # Helpers
    # ----------------------------
    def _set_label(self, name: str, text: str):
        if hasattr(self._widget, name):
            getattr(self._widget, name).setText(text)

    def _sync_all_value_labels(self):
        # Initialize command display from sliders
        self.onPositionMove(int(self._widget.position.value()))
        self.onVelocityMove(int(self._widget.velocity.value()))
        self.onTorqueMove(int(self._widget.torque.value()))

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


def main():
    Main().main(sys.argv, standalone="rqt_armcontrol.rqt_actuatorcontrol")


if __name__ == "__main__":
    main()
