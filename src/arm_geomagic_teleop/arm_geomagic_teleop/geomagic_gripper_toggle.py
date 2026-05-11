#!/usr/bin/env python3

from enum import Enum
from typing import List

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class GripperState(Enum):
    OPEN = "open"
    CLOSED = "closed"


class GeomagicGripperToggle(Node):
    """Toggle the gripper open/closed on a rising edge of a Geomagic button."""

    def __init__(self):
        super().__init__("geomagic_gripper_toggle")

        self.declare_parameter("input_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("status_topic", "/arm_teleop/gripper_status")
        self.declare_parameter("gripper_action_name", "/gripper_controller/gripper_cmd")
        self.declare_parameter("toggle_button_index", 1)
        self.declare_parameter("open_position", 0.0)
        self.declare_parameter("closed_position", -0.69)
        self.declare_parameter("max_effort", 0.0)
        self.declare_parameter("start_closed", False)
        self.declare_parameter("debounce_s", 0.35)
        self.declare_parameter("wait_for_server_s", 0.0)

        self.input_buttons_topic = self.get_parameter("input_buttons_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.gripper_action_name = self.get_parameter("gripper_action_name").value
        self.toggle_button_index = int(self.get_parameter("toggle_button_index").value)
        self.open_position = float(self.get_parameter("open_position").value)
        self.closed_position = float(self.get_parameter("closed_position").value)
        self.max_effort = float(self.get_parameter("max_effort").value)
        self.state = (
            GripperState.CLOSED
            if as_bool(self.get_parameter("start_closed").value)
            else GripperState.OPEN
        )
        self.debounce_s = float(self.get_parameter("debounce_s").value)
        self.wait_for_server_s = float(self.get_parameter("wait_for_server_s").value)

        self.buttons: List[int] = []
        self.previous_button = False
        self.last_toggle_time = self.get_clock().now() - Duration(seconds=10.0)
        self.goal_in_flight = False

        self.action_client = ActionClient(self, GripperCommand, self.gripper_action_name)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)

        self.get_logger().warn(
            "Geomagic gripper toggle ready: "
            f"button={self.toggle_button_index}, action={self.gripper_action_name}, "
            f"open={self.open_position:.3f}, close={self.closed_position:.3f}"
        )

    def buttons_callback(self, msg: Joy) -> None:
        self.buttons = [1 if value else 0 for value in msg.buttons]
        pressed = (
            0 <= self.toggle_button_index < len(self.buttons)
            and bool(self.buttons[self.toggle_button_index])
        )

        if pressed and not self.previous_button:
            self.handle_rising_edge()

        self.previous_button = pressed

    def handle_rising_edge(self) -> None:
        now = self.get_clock().now()
        if now - self.last_toggle_time < Duration(seconds=self.debounce_s):
            self.publish_status("debounced")
            return
        if self.goal_in_flight:
            self.publish_status("busy")
            return

        next_state = GripperState.CLOSED if self.state == GripperState.OPEN else GripperState.OPEN
        target = self.closed_position if next_state == GripperState.CLOSED else self.open_position

        if not self.action_client.wait_for_server(timeout_sec=self.wait_for_server_s):
            self.get_logger().warn(
                f"Gripper action server {self.gripper_action_name} is not available."
            )
            self.publish_status("no_action_server")
            return

        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = self.max_effort

        self.goal_in_flight = True
        self.last_toggle_time = now
        self.publish_status(f"sending_{next_state.value}")
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(lambda done_future: self.goal_response_callback(done_future, next_state))

    def goal_response_callback(self, future, next_state: GripperState) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_in_flight = False
            self.get_logger().warn("Gripper goal rejected.")
            self.publish_status("rejected")
            return

        self.state = next_state
        self.publish_status(f"accepted_{self.state.value}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        self.goal_in_flight = False
        result = future.result()
        self.publish_status(f"done_{self.state.value}_status_{result.status}")

    def publish_status(self, state: str) -> None:
        msg = String()
        msg.data = (
            f"state={state}; gripper={self.state.value}; "
            f"button_index={self.toggle_button_index}; action={self.gripper_action_name}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = GeomagicGripperToggle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
