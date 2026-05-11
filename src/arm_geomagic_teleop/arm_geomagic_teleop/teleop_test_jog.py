#!/usr/bin/env python3

import math
from typing import List

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy


def axis_vector(axis: str, speed: float) -> List[float]:
    values = [0.0, 0.0, 0.0]
    token = axis.strip().lower()
    sign = -1.0 if token.startswith("-") else 1.0
    name = token[1:] if token.startswith("-") else token
    index_by_name = {"x": 0, "y": 1, "z": 2}
    if name not in index_by_name:
        raise ValueError("axis must be one of x, y, z, -x, -y, -z")
    values[index_by_name[name]] = sign * abs(speed)
    return values


class TeleopTestJog(Node):
    """Publish a synthetic deadman + raw twist to test the teleop pipeline."""

    def __init__(self):
        super().__init__("teleop_test_jog")

        self.declare_parameter("raw_twist_topic", "/arm_teleop/raw_twist_cmd")
        self.declare_parameter("buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("command_frame", "uav_mount")
        self.declare_parameter("axis", "x")
        self.declare_parameter("speed_m_s", 0.05)
        self.declare_parameter("duration_s", 2.0)
        self.declare_parameter("publish_rate_hz", 50.0)

        self.raw_twist_topic = self.get_parameter("raw_twist_topic").value
        self.buttons_topic = self.get_parameter("buttons_topic").value
        self.command_frame = self.get_parameter("command_frame").value
        self.axis = self.get_parameter("axis").value
        self.speed_m_s = float(self.get_parameter("speed_m_s").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.linear = axis_vector(self.axis, self.speed_m_s)
        self.started_at = self.get_clock().now()
        self.done = False

        self.twist_pub = self.create_publisher(TwistStamped, self.raw_twist_topic, 10)
        self.buttons_pub = self.create_publisher(Joy, self.buttons_topic, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().warn(
            f"Publishing synthetic jog: axis={self.axis}, speed={self.speed_m_s:.3f} m/s, "
            f"duration={self.duration_s:.2f}s"
        )

    def timer_callback(self) -> None:
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1e-9
        active = elapsed <= self.duration_s

        buttons = Joy()
        buttons.header.stamp = self.get_clock().now().to_msg()
        buttons.buttons = [1 if active else 0, 0]
        self.buttons_pub.publish(buttons)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame
        if active:
            twist.twist.linear.x = self.linear[0]
            twist.twist.linear.y = self.linear[1]
            twist.twist.linear.z = self.linear[2]
        self.twist_pub.publish(twist)

        if not active and not self.done:
            self.done = True
            self.get_logger().warn("Synthetic jog complete; published released deadman and zero twist.")


def main():
    rclpy.init()
    try:
        node = TeleopTestJog()
    except ValueError as exc:
        node = Node("teleop_test_jog_error")
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return

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
