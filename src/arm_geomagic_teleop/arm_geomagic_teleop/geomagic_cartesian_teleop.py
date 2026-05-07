#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def clamp_unit(value: float) -> float:
    return max(0.0, min(1.0, value))


def clamp_vector(values: List[float], max_norm: float) -> List[float]:
    if max_norm <= 0.0:
        return [0.0 for _ in values]
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= max_norm or norm <= 1e-12:
        return values
    scale = max_norm / norm
    return [value * scale for value in values]


def apply_deadband(value: float, deadband: float) -> float:
    magnitude = abs(value)
    if magnitude <= max(0.0, deadband):
        return 0.0
    result = math.copysign(magnitude - deadband, value)
    return 0.0 if abs(result) <= 1e-12 else result


def normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def inverse_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = normalize_quaternion(q)
    return (-x, -y, -z, w)


def multiply_quaternion(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = lhs
    x2, y2, z2, w2 = rhs
    return normalize_quaternion(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
    )


def quaternion_to_rotvec(q: Tuple[float, float, float, float]) -> List[float]:
    x, y, z, w = normalize_quaternion(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half <= 1e-9:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(sin_half, w)
    return [angle * x / sin_half, angle * y / sin_half, angle * z / sin_half]


def parse_axis_token(token: str) -> Tuple[int, float]:
    token = token.strip().lower()
    sign = -1.0 if token.startswith("-") else 1.0
    axis = token[1:] if token.startswith("-") else token
    axis_map = {"x": 0, "y": 1, "z": 2}
    if axis not in axis_map:
        raise ValueError(f"Invalid axis token '{token}'.")
    return axis_map[axis], sign


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clean_near_zero(value: float) -> float:
    return 0.0 if abs(value) <= 1e-12 else value


class GeomagicCartesianTeleop(Node):
    """Convert deadman-clutched Geomagic pose displacement into bounded Cartesian twist."""

    def __init__(self):
        super().__init__("geomagic_cartesian_teleop")

        self.declare_parameter("input_pose_topic", "/geomagic_touch/pose")
        self.declare_parameter("input_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("output_twist_topic", "/arm_teleop/raw_twist_cmd")
        self.declare_parameter("status_topic", "/arm_teleop/teleop_status")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("ee_frame_name", "tool0")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("deadman_button_index", 0)
        self.declare_parameter("translation_gain", 1.0)
        self.declare_parameter("translation_deadband_m", 0.004)
        self.declare_parameter("rotation_gain", 1.0)
        self.declare_parameter("orientation_deadband_rad", 0.05)
        self.declare_parameter("max_linear_speed_m_s", 0.02)
        self.declare_parameter("max_angular_speed_rad_s", 0.10)
        self.declare_parameter("low_pass_alpha", 0.45)
        self.declare_parameter("orientation_enabled", False)
        self.declare_parameter("robot_axes_from_device", ["y", "-x", "z"])
        self.declare_parameter("robot_angular_axes_from_device", ["y", "-x", "z"])
        self.declare_parameter("zero_on_release", True)

        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.input_buttons_topic = self.get_parameter("input_buttons_topic").value
        self.output_twist_topic = self.get_parameter("output_twist_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.command_frame = self.get_parameter("command_frame").value
        self.ee_frame_name = self.get_parameter("ee_frame_name").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.translation_gain = float(self.get_parameter("translation_gain").value)
        self.translation_deadband = float(self.get_parameter("translation_deadband_m").value)
        self.rotation_gain = float(self.get_parameter("rotation_gain").value)
        self.orientation_deadband = float(self.get_parameter("orientation_deadband_rad").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed_m_s").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed_rad_s").value)
        self.low_pass_alpha = clamp_unit(float(self.get_parameter("low_pass_alpha").value))
        self.orientation_enabled = as_bool(self.get_parameter("orientation_enabled").value)
        self.zero_on_release = as_bool(self.get_parameter("zero_on_release").value)
        self.axis_map = [parse_axis_token(token) for token in self.get_parameter("robot_axes_from_device").value]
        self.angular_axis_map = [
            parse_axis_token(token) for token in self.get_parameter("robot_angular_axes_from_device").value
        ]

        self.latest_pose: Optional[PoseStamped] = None
        self.latest_pose_time = self.get_clock().now()
        self.buttons: List[int] = []
        self.clutch_anchor: Optional[PoseStamped] = None
        self.filtered_linear = [0.0, 0.0, 0.0]
        self.filtered_angular = [0.0, 0.0, 0.0]
        self.last_deadman = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, self.output_twist_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(PoseStamped, self.input_pose_topic, self.pose_callback, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().warn(
            f"Geomagic teleop ready: deadman={self.deadman_button_index}, "
            f"linear_limit={self.max_linear_speed:.3f} m/s, "
            f"angular_limit={self.max_angular_speed:.3f} rad/s"
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.latest_pose_time = self.get_clock().now()

    def buttons_callback(self, msg: Joy) -> None:
        self.buttons = [1 if value else 0 for value in msg.buttons]

    def deadman_active(self) -> bool:
        return 0 <= self.deadman_button_index < len(self.buttons) and bool(
            self.buttons[self.deadman_button_index]
        )

    def input_fresh(self) -> bool:
        return self.latest_pose is not None and (
            self.get_clock().now() - self.latest_pose_time
        ) < Duration(seconds=self.input_timeout_s)

    def timer_callback(self) -> None:
        deadman = self.deadman_active()
        fresh = self.input_fresh()

        if deadman and fresh:
            if not self.last_deadman or self.clutch_anchor is None:
                self.clutch_anchor = self.latest_pose
                self.filtered_linear = [0.0, 0.0, 0.0]
                self.filtered_angular = [0.0, 0.0, 0.0]
                self.publish_status("clutched")
            self.publish_motion_command()
        else:
            self.clutch_anchor = None
            self.filtered_linear = [0.0, 0.0, 0.0]
            self.filtered_angular = [0.0, 0.0, 0.0]
            if self.zero_on_release:
                self.publish_twist(self.filtered_linear, self.filtered_angular)
            self.publish_status("idle" if fresh else "waiting_for_input")

        self.last_deadman = deadman

    def publish_motion_command(self) -> None:
        if self.latest_pose is None or self.clutch_anchor is None:
            return

        current = self.latest_pose.pose
        anchor = self.clutch_anchor.pose
        device_delta = [
            current.position.x - anchor.position.x,
            current.position.y - anchor.position.y,
            current.position.z - anchor.position.z,
        ]
        linear = []
        for index, sign in self.axis_map:
            linear.append(sign * apply_deadband(device_delta[index], self.translation_deadband))
        linear = [self.translation_gain * value for value in linear]
        linear = clamp_vector(linear, self.max_linear_speed)

        angular = [0.0, 0.0, 0.0]
        if self.orientation_enabled:
            angular = self.compute_angular_command(current, anchor)

        alpha = self.low_pass_alpha
        self.filtered_linear = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(linear, self.filtered_linear)
        ]
        self.filtered_angular = [
            alpha * target + (1.0 - alpha) * previous
            for target, previous in zip(angular, self.filtered_angular)
        ]
        self.publish_twist(self.filtered_linear, self.filtered_angular)
        self.publish_status("moving")

    def compute_angular_command(self, current, anchor) -> List[float]:
        q_current = (
            current.orientation.x,
            current.orientation.y,
            current.orientation.z,
            current.orientation.w,
        )
        q_anchor = (
            anchor.orientation.x,
            anchor.orientation.y,
            anchor.orientation.z,
            anchor.orientation.w,
        )
        device_rotvec = quaternion_to_rotvec(
            multiply_quaternion(inverse_quaternion(q_anchor), q_current)
        )
        angular = []
        for index, sign in self.angular_axis_map:
            angular.append(sign * apply_deadband(device_rotvec[index], self.orientation_deadband))
        angular = [self.rotation_gain * value for value in angular]
        return clamp_vector(angular, self.max_angular_speed)

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

    def publish_status(self, state: str) -> None:
        msg = String()
        msg.data = (
            f"state={state}; deadman={int(self.deadman_active())}; "
            f"orientation={int(self.orientation_enabled)}; output={self.output_twist_topic}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = GeomagicCartesianTeleop()
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
