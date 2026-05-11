#!/usr/bin/env python3

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8
from tf2_ros import Buffer, TransformException, TransformListener


ALL_READY = 4


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def clamp_vector(values: List[float], max_norm: float) -> List[float]:
    if max_norm <= 0.0:
        return [0.0 for _ in values]
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= max_norm or norm <= 1e-12:
        return values
    scale = max_norm / norm
    return [value * scale for value in values]


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clean_near_zero(value: float) -> float:
    return 0.0 if abs(value) <= 1e-12 else value


def vector_norm(values: List[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


class TeleopSafetyFilter(Node):
    """Gate and smooth teleop twist commands before MoveIt Servo."""

    def __init__(self):
        super().__init__("teleop_safety_filter")

        self.declare_parameter("input_twist_topic", "/arm_teleop/raw_twist_cmd")
        self.declare_parameter("output_twist_topic", "/arm_teleop/twist_cmd")
        self.declare_parameter("input_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("homing_state_topic", "/arm_homing/state")
        self.declare_parameter("status_topic", "/arm_teleop/safety_status")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("ee_frame_name", "tool0")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("button_timeout_s", 0.25)
        self.declare_parameter("homing_timeout_s", 1.0)
        self.declare_parameter("deadman_button_index", 0)
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("require_homing", True)
        self.declare_parameter("require_tool_tf", True)
        self.declare_parameter("low_pass_alpha", 0.35)
        self.declare_parameter("max_linear_accel_m_s2", 0.04)
        self.declare_parameter("max_angular_accel_rad_s2", 0.20)
        self.declare_parameter("workspace_min", [-0.60, -0.80, -0.90])
        self.declare_parameter("workspace_max", [0.80, 0.80, 0.20])
        self.declare_parameter("max_linear_m_s", 0.02)
        self.declare_parameter("max_angular_rad_s", 0.10)

        self.input_twist_topic = self.get_parameter("input_twist_topic").value
        self.output_twist_topic = self.get_parameter("output_twist_topic").value
        self.input_buttons_topic = self.get_parameter("input_buttons_topic").value
        self.homing_state_topic = self.get_parameter("homing_state_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.command_frame = self.get_parameter("command_frame").value
        self.ee_frame_name = self.get_parameter("ee_frame_name").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.button_timeout_s = float(self.get_parameter("button_timeout_s").value)
        self.homing_timeout_s = float(self.get_parameter("homing_timeout_s").value)
        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.require_deadman = as_bool(self.get_parameter("require_deadman").value)
        self.require_homing = as_bool(self.get_parameter("require_homing").value)
        self.require_tool_tf = as_bool(self.get_parameter("require_tool_tf").value)
        self.low_pass_alpha = max(0.0, min(1.0, float(self.get_parameter("low_pass_alpha").value)))
        self.max_linear_accel = max(0.0, float(self.get_parameter("max_linear_accel_m_s2").value))
        self.max_angular_accel = max(0.0, float(self.get_parameter("max_angular_accel_rad_s2").value))
        self.workspace_min = self.read_vector_param("workspace_min")
        self.workspace_max = self.read_vector_param("workspace_max")
        self.max_linear = float(self.get_parameter("max_linear_m_s").value)
        self.max_angular = float(self.get_parameter("max_angular_rad_s").value)

        self.latest_twist: Optional[TwistStamped] = None
        self.latest_twist_time = self.get_clock().now()
        self.buttons: List[int] = []
        self.latest_button_time = self.get_clock().now()
        self.homing_state: Optional[int] = None
        self.latest_homing_time = self.get_clock().now()
        self.filtered_linear = [0.0, 0.0, 0.0]
        self.filtered_angular = [0.0, 0.0, 0.0]
        self.last_target_linear = [0.0, 0.0, 0.0]
        self.last_target_angular = [0.0, 0.0, 0.0]
        self.last_publish_time = self.get_clock().now()
        self.last_reason = "startup_hold"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, self.output_twist_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(TwistStamped, self.input_twist_topic, self.twist_callback, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)
        self.create_subscription(UInt8, self.homing_state_topic, self.homing_callback, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().warn(
            f"Teleop safety filter ready: homing_required={self.require_homing}, "
            f"workspace={self.workspace_min}..{self.workspace_max}, "
            f"linear_limit={self.max_linear:.3f}, angular_limit={self.max_angular:.3f}"
        )

    def read_vector_param(self, name: str) -> List[float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must have exactly three values")
        return [float(value) for value in values]

    def twist_callback(self, msg: TwistStamped) -> None:
        self.latest_twist = msg
        self.latest_twist_time = self.get_clock().now()

    def buttons_callback(self, msg: Joy) -> None:
        self.buttons = [1 if value else 0 for value in msg.buttons]
        self.latest_button_time = self.get_clock().now()

    def homing_callback(self, msg: UInt8) -> None:
        self.homing_state = int(msg.data)
        self.latest_homing_time = self.get_clock().now()

    def deadman_active(self) -> bool:
        if not self.require_deadman:
            return True
        fresh = (self.get_clock().now() - self.latest_button_time) < Duration(
            seconds=self.button_timeout_s
        )
        return (
            fresh
            and 0 <= self.deadman_button_index < len(self.buttons)
            and bool(self.buttons[self.deadman_button_index])
        )

    def homing_ready(self) -> bool:
        if not self.require_homing:
            return True
        fresh = (self.get_clock().now() - self.latest_homing_time) < Duration(
            seconds=self.homing_timeout_s
        )
        return fresh and self.homing_state == ALL_READY

    def twist_fresh(self) -> bool:
        return self.latest_twist is not None and (
            self.get_clock().now() - self.latest_twist_time
        ) < Duration(seconds=self.input_timeout_s)

    def get_tool_position(self) -> Optional[List[float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.command_frame,
                self.ee_frame_name,
                Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.command_frame}->{self.ee_frame_name}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        t = transform.transform.translation
        return [t.x, t.y, t.z]

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_publish_time).nanoseconds * 1e-9, 1.0 / self.publish_rate_hz)
        self.last_publish_time = now

        linear, angular, reason = self.compute_target()
        self.last_target_linear = list(linear)
        self.last_target_angular = list(angular)
        linear = self.apply_accel_limit(self.filtered_linear, linear, self.max_linear_accel, dt)
        angular = self.apply_accel_limit(self.filtered_angular, angular, self.max_angular_accel, dt)

        alpha = self.low_pass_alpha
        self.filtered_linear = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(linear, self.filtered_linear)
        ]
        self.filtered_angular = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(angular, self.filtered_angular)
        ]
        self.last_reason = reason
        self.publish_twist(self.filtered_linear, self.filtered_angular)
        self.publish_status()

    def compute_target(self) -> tuple[List[float], List[float], str]:
        if not self.homing_ready():
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "homing_not_ready"
        if not self.deadman_active():
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "deadman_released"
        if not self.twist_fresh() or self.latest_twist is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "input_timeout"

        twist = self.latest_twist.twist
        linear = clamp_vector([twist.linear.x, twist.linear.y, twist.linear.z], self.max_linear)
        angular = clamp_vector([twist.angular.x, twist.angular.y, twist.angular.z], self.max_angular)

        tool_position = self.get_tool_position()
        if tool_position is None:
            if self.require_tool_tf:
                return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "missing_tool_tf"
            return linear, angular, "allowed_without_workspace"

        linear = self.apply_workspace_limits(tool_position, linear)
        if all(abs(value) <= 1e-9 for value in linear) and any(abs(value) > 1e-9 for value in angular):
            return linear, angular, "allowed_rotation_only"
        if all(abs(value) <= 1e-9 for value in linear) and any(
            abs(value) > 1e-9 for value in [twist.linear.x, twist.linear.y, twist.linear.z]
        ):
            return linear, angular, "workspace_boundary"
        return linear, angular, "allowed"

    def apply_workspace_limits(self, position: List[float], linear: List[float]) -> List[float]:
        limited = list(linear)
        for index in range(3):
            if position[index] <= self.workspace_min[index] and limited[index] < 0.0:
                limited[index] = 0.0
            if position[index] >= self.workspace_max[index] and limited[index] > 0.0:
                limited[index] = 0.0
        return limited

    def apply_accel_limit(
        self, previous: List[float], target: List[float], accel_limit: float, dt: float
    ) -> List[float]:
        if accel_limit <= 0.0:
            return target
        max_delta = accel_limit * dt
        return [
            previous_value + clamp(target_value - previous_value, max_delta)
            for previous_value, target_value in zip(previous, target)
        ]

    def publish_twist(self, linear: List[float], angular: List[float]) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = clean_near_zero(linear[0])
        msg.twist.linear.y = clean_near_zero(linear[1])
        msg.twist.linear.z = clean_near_zero(linear[2])
        msg.twist.angular.x = clean_near_zero(angular[0])
        msg.twist.angular.y = clean_near_zero(angular[1])
        msg.twist.angular.z = clean_near_zero(angular[2])
        self.twist_pub.publish(msg)

    def publish_status(self) -> None:
        raw_linear = [0.0, 0.0, 0.0]
        raw_angular = [0.0, 0.0, 0.0]
        if self.latest_twist is not None:
            twist = self.latest_twist.twist
            raw_linear = [twist.linear.x, twist.linear.y, twist.linear.z]
            raw_angular = [twist.angular.x, twist.angular.y, twist.angular.z]
        msg = String()
        msg.data = (
            f"homed={int(self.homing_ready())}; homing_state={self.homing_state}; "
            f"deadman={int(self.deadman_active())}; raw_fresh={int(self.twist_fresh())}; "
            f"reason={self.last_reason}; raw_linear_norm={vector_norm(raw_linear):.6f}; "
            f"target_linear_norm={vector_norm(self.last_target_linear):.6f}; "
            f"filtered_linear_norm={vector_norm(self.filtered_linear):.6f}; "
            f"raw_angular_norm={vector_norm(raw_angular):.6f}; "
            f"target_angular_norm={vector_norm(self.last_target_angular):.6f}; "
            f"output={self.output_twist_topic}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopSafetyFilter()
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
