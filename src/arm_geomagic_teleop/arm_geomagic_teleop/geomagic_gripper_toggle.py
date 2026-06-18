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
        self.declare_parameter("allow_interrupt", True)

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
        self.allow_interrupt = as_bool(self.get_parameter("allow_interrupt").value)

        self.buttons: List[int] = []
        self.previous_button = False
        self.last_toggle_time = self.get_clock().now() - Duration(seconds=10.0)
        self.goal_in_flight = False
        self.active_goal_handle = None
        self.goal_counter = 0
        self.active_goal_id = 0

        self.action_client = ActionClient(self, GripperCommand, self.gripper_action_name)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)

        self.get_logger().warn(
            "Geomagic gripper toggle ready: "
            f"button={self.toggle_button_index}, action={self.gripper_action_name}, "
            f"open={self.open_position:.3f}, close={self.closed_position:.3f}, "
            f"allow_interrupt={self.allow_interrupt}"
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
            if not self.allow_interrupt:
                self.publish_status("busy")
                return
            self.publish_status(f"interrupting_{self.state.value}")
            self.cancel_active_goal()

        previous_state = self.state
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

        self.goal_counter += 1
        goal_id = self.goal_counter
        self.active_goal_id = goal_id
        self.active_goal_handle = None
        self.goal_in_flight = True
        self.state = next_state
        self.last_toggle_time = now
        self.publish_status(f"sending_{next_state.value}")
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done_future: self.goal_response_callback(
                done_future, goal_id, next_state, previous_state
            )
        )

    def cancel_active_goal(self) -> None:
        goal_handle = self.active_goal_handle
        goal_id = self.active_goal_id
        if goal_handle is None:
            self.publish_status("interrupt_pending_goal_response")
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 - ROS action futures can raise middleware errors.
            self.get_logger().warn(f"Failed to request gripper goal cancel: {exc}")
            self.publish_status("cancel_request_failed")
            return

        cancel_future.add_done_callback(
            lambda done_future: self.cancel_response_callback(done_future, goal_id)
        )
        self.publish_status("cancel_requested")

    def goal_response_callback(
        self,
        future,
        goal_id: int,
        next_state: GripperState,
        previous_state: GripperState,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS action futures can raise middleware errors.
            if goal_id == self.active_goal_id:
                self.goal_in_flight = False
                self.active_goal_handle = None
                self.state = previous_state
            self.get_logger().warn(f"Gripper goal send failed: {exc}")
            self.publish_status("send_failed")
            return

        if goal_id != self.active_goal_id:
            if goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"Failed to cancel stale gripper goal: {exc}")
            self.publish_status("stale_goal_response")
            return

        if not goal_handle.accepted:
            self.goal_in_flight = False
            self.active_goal_handle = None
            self.state = previous_state
            self.get_logger().warn("Gripper goal rejected.")
            self.publish_status("rejected")
            return

        self.active_goal_handle = goal_handle
        self.publish_status(f"accepted_{self.state.value}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self.result_callback(done_future, goal_id, next_state)
        )

    def cancel_response_callback(self, future, goal_id: int) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Gripper cancel response failed: {exc}")
            if goal_id == self.active_goal_id:
                self.publish_status("cancel_failed")
            return

        if goal_id == self.active_goal_id:
            self.publish_status("cancel_accepted")

    def result_callback(self, future, goal_id: int, next_state: GripperState) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - keep node alive on action errors.
            if goal_id == self.active_goal_id:
                self.goal_in_flight = False
                self.active_goal_handle = None
            self.get_logger().warn(f"Gripper result failed: {exc}")
            self.publish_status("result_failed")
            return

        if goal_id != self.active_goal_id:
            self.publish_status("stale_result")
            return

        self.goal_in_flight = False
        self.active_goal_handle = None
        self.state = next_state
        self.publish_status(f"done_{self.state.value}_status_{result.status}")

    def publish_status(self, state: str) -> None:
        msg = String()
        msg.data = (
            f"state={state}; gripper={self.state.value}; "
            f"in_flight={self.goal_in_flight}; goal_id={self.active_goal_id}; "
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
