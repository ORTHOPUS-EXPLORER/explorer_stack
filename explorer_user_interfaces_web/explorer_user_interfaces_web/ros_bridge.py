#!/usr/bin/env python3

import asyncio
import os
import threading
import time
from typing import Dict, List

import yaml
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32, String

from orthopus_vesc_interfaces.msg import State

NUM_JOINTS = 6

# Updates waiting to be sent to a page
CLIENT_QUEUE_SIZE = 256

# Must match orthopus_vesc_interface's state_pub_ QoS (transient-local, depth 1)
# so we get the current state immediately on connect instead of waiting for
# the next change.
QOS_JOINT_STATE = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# Must match command_node's latched publishers QoS (transient-local, depth 1): speed level and
# retract status are only published on change.
QOS_COMMAND_NODE_LATCH_PROFILE = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class RosBridge:
    """Bridge between ROS2 and the web interface"""

    def __init__(self, ros_node: Node):
        self.node = ros_node
        self.current_mode = "Unknown"
        self.speed_level = None
        self.retract_status = (
            "retracted"  # Retract status: ready, in progress, retracted
        )
        self.drink_active = False
        self.gripper_lock_active = False
        self.gripper_closure_direction: str | None = None
        # Queue of updates of each connected page, only used from the server event loop
        self.connected_clients: set[asyncio.Queue] = set()
        self._event_loop: asyncio.AbstractEventLoop | None = None

        # Load mode config once at startup
        config_path = (
            ros_node.get_parameter("mode_config_path").value
            if ros_node.has_parameter("mode_config_path")
            else None
        )
        self.mode_config = load_mode_config(config_path)

        # ROS2 subscribers
        self.mode_subscriber = self.node.create_subscription(
            String, "/command_node/mode_name", self.mode_callback, 10
        )

        # Speed level subscriber - integer topic
        self.speed_level_subscriber = self.node.create_subscription(
            Int32,
            "/command_node/speed_level",
            self.speed_level_callback,
            QOS_COMMAND_NODE_LATCH_PROFILE,
        )

        # Retract status subscriber - string topic
        self.retract_status_subscriber = self.node.create_subscription(
            String,
            "/command_node/retract_status",
            self.retract_status_callback,
            QOS_COMMAND_NODE_LATCH_PROFILE,
        )

        # Joint state subscribers - one per joint, published only on actual change
        self.joint_state_subscribers = [
            self.node.create_subscription(
                State,
                f"/explorer_joint_{joint_index}/state",
                lambda msg, joint_index=joint_index: self.joint_state_callback(
                    joint_index, msg
                ),
                QOS_JOINT_STATE,
            )
            for joint_index in range(1, NUM_JOINTS + 1)
        ]
        self.last_joint_state_message_dict = {}

        self.gripper_lock_subscriber = self.node.create_subscription(
            Bool,
            "/explorer_input_devices/joystick/gripper_lock",
            self.gripper_lock_callback,
            10,
        )

        # Camera related variables
        camera_topic = ros_node.get_parameter("camera_topic").value
        self._camera_frame_condition = threading.Condition()
        self._latest_camera_frame: bytes | None = None
        self._latest_camera_frame_index = 0
        self.camera_subscriber = self.node.create_subscription(
            CompressedImage,
            camera_topic,
            self.camera_image_callback,
            qos_profile_sensor_data,
        )

        self.gripper_closure_direction_publisher = self.node.create_publisher(
           String,
           "/gripper/closure_direction",
           10,
        )


        self.node.get_logger().info("ROS Bridge initialized")


    def set_gripper_closure_direction(self, direction: str):
        """Publish the selected gripper closure direction."""


        valid_directions = {"bas", "haut", "droite", "gauche"}


        if direction not in valid_directions:
            self.node.get_logger().warning(f"Invalid gripper direction: {direction}")
            return False


        msg = String()
        msg.data = direction


        self.gripper_closure_direction = direction
        self.gripper_closure_direction_publisher.publish(msg)


        self.node.get_logger().info(f"Gripper closure direction published: {direction}")

        self._publish_update(
            {
                "type": "gripper_closure_direction_update",
                "direction": direction,
                "timestamp": time.time(),
            }
        )


        return True


    def mode_callback(self, msg: String):
        """Callback for mode changes"""
        new_mode = msg.data
        # Only process if mode actually changed
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            self.node.get_logger().info(f"Mode changed to: {self.current_mode}")
            # Check drink axis in config
            drink_active = False
            # Assume self.mode_config is loaded and available
            button_mappings = getattr(self, "mode_config", {}).get(
                "button_mappings", {}
            )
            mapping = button_mappings.get(new_mode, {})
            axes = mapping.get("axes", [])
            for axis in axes:
                if axis.get("control_name", "").lower() == "drink":
                    drink_active = True
                    break
            print(f"[DEBUG] Backend: drink_active for mode {new_mode}: {drink_active}")
            self.drink_active = drink_active
            # Put update in queue for drink LED
            self._publish_update(
                {
                    "type": "drink_led_update",
                    "active": drink_active,
                    "mode": self.current_mode,
                    "timestamp": time.time(),
                }
            )
            # Put update in queue for mode
            self._publish_update(
                {
                    "type": "mode_update",
                    "mode": self.current_mode,
                    "timestamp": time.time(),
                }
            )

    def speed_level_callback(self, msg: Int32):
        """Callback for speed level changes"""
        new_level = msg.data
        if new_level != self.speed_level:
            self.speed_level = new_level
            self.node.get_logger().info(f"Speed level changed to: {self.speed_level}")
            self._publish_update(
                {
                    "type": "speed_level_update",
                    "speed_level": self.speed_level,
                    "timestamp": time.time(),
                }
            )

    def retract_status_callback(self, msg: String):
        """Callback for retract status changes"""
        new_status = msg.data
        if new_status != self.retract_status:
            self.retract_status = new_status
            self.node.get_logger().info(
                f"Retract status changed to: {self.retract_status}"
            )
            self._publish_update(
                {
                    "type": "retract_status_update",
                    "retract_status": self.retract_status,
                    "timestamp": time.time(),
                }
            )

    def joint_state_callback(self, joint_index: int, msg: State):
        """Callback for joint state feedback."""
        self.last_joint_state_message_dict[joint_index] = {
            "type": "joint_state_update",
            "joint_index": joint_index,
            "joint_name": msg.joint_name,
            "mode": msg.mode,
            "state": msg.state,
            "error": msg.error,
            "timestamp": time.time(),
        }
        self._publish_update(self.last_joint_state_message_dict[joint_index])

    def gripper_lock_callback(self, msg: Bool):
        """Callback for gripper lock state changes."""
        new_state = msg.data

        print(f"[DEBUG] gripper_lock reçu : {new_state}")

        if new_state != self.gripper_lock_active:
            self.gripper_lock_active = new_state

            self.node.get_logger().info(
                f"Gripper lock changed to: {self.gripper_lock_active}"
            )

            self._publish_update(
                {
                    "type": "gripper_lock_led_update",
                    "active": self.gripper_lock_active,
                    "timestamp": time.time(),
                }
            )

    def camera_image_callback(self, msg: CompressedImage):
        """Callback for ROS camera frame topic"""
        with self._camera_frame_condition:
            self._latest_camera_frame = bytes(msg.data)
            self._latest_camera_frame_index += 1
            self._camera_frame_condition.notify_all()

    def _wait_for_camera_frame(self, last_frame_index: int, timeout: float):
        """Blocking wait (should run in an executor thread) until a frame newer than
        last_frame_index arrives, or timeout occured.
        """
        with self._camera_frame_condition:
            self._camera_frame_condition.wait_for(
                lambda: (
                    self._latest_camera_frame is not None
                    and self._latest_camera_frame_index > last_frame_index
                ),
                timeout=timeout,
            )
            return self._latest_camera_frame, self._latest_camera_frame_index

    async def get_latest_camera_frame(
        self, last_frame_index: int, timeout: float = 5.0
    ):
        """Return a tuple (frame, last_frame_index) or (None, last_frame_index)
        if timeout occured.
        """
        loop = asyncio.get_running_loop()
        frame, frame_index = await loop.run_in_executor(
            None, self._wait_for_camera_frame, last_frame_index, timeout
        )
        if frame_index == last_frame_index:
            return None, last_frame_index
        return frame, frame_index

    def set_event_loop(self, event_loop: asyncio.AbstractEventLoop):
        """Set the server event loop, which updates are handed over to from the ROS thread."""
        self._event_loop = event_loop

    def add_client(self) -> asyncio.Queue:
        """Register a connected page, must be called from the server event loop.

        Returns:
            asyncio.Queue: updates to send to this page, starting with the current state
        """
        client_queue = asyncio.Queue(maxsize=CLIENT_QUEUE_SIZE)
        self.connected_clients.add(client_queue)
        self._put_current_state(client_queue)
        return client_queue

    def remove_client(self, client_queue: asyncio.Queue):
        """Unregister a page, must be called from the server event loop."""
        self.connected_clients.discard(client_queue)

    def _publish_update(self, update: Dict):
        """Send an update to every connected page, can be called from any thread."""
        if self._event_loop is None:
            # Server not started, so no page yet: it gets the current state when it connects
            return
        try:
            self._event_loop.call_soon_threadsafe(self._dispatch_update, update)
        except RuntimeError:
            # Event loop closed: the server is shutting down
            pass

    def _dispatch_update(self, update: Dict):
        """Add an update to the queue of every page, runs in the server event loop."""
        for client_queue in self.connected_clients:
            try:
                client_queue.put_nowait(update)
            except asyncio.QueueFull:
                # Page too slow (e.g. sleeping device): its backlog is replaced by the current
                # state, which already includes this update
                while not client_queue.empty():
                    client_queue.get_nowait()
                self._put_current_state(client_queue)

    def _put_current_state(self, client_queue: asyncio.Queue):
        for message in self.get_initial_message_list_on_connected():
            client_queue.put_nowait(message)

    def get_initial_message_list_on_connected(self) -> List[Dict]:
        initial_messages = [
            {
                "type": "initial",
                "mode": self.current_mode,
                "speed_level": self.speed_level,
                "retract_status": self.retract_status,
                "drink_active": self.drink_active,
                "gripper_lock_active": self.gripper_lock_active,
                "gripper_closure_direction": self.gripper_closure_direction,
            }
        ]

        # Copy: joints are added from the ROS thread
        for joint_state_message in list(self.last_joint_state_message_dict.values()):
            initial_messages.append(joint_state_message)

        return initial_messages


def load_mode_config(config_path: str) -> Dict:
    """Load mode configuration from YAML file"""
    if not config_path or not os.path.exists(config_path):
        return {
            "mode_info": {
                "name": "Unknown",
                "display_name": "No Configuration",
                "description": "Configuration file not found",
                "default_image": "default.png",
            },
            "button_mappings": {},
        }

    try:
        with open(config_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config: {e}")
        return {"mode_info": {}, "button_mappings": {}}
