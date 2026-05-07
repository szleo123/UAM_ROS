#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class GeomagicOmniStateAdapter(Node):
    """Adapt Geomagic Touch OmniState to PoseStamped and Joy button topics."""

    def __init__(self):
        super().__init__("geomagic_omni_state_adapter")

        self.declare_parameter("omni_state_topic", "/phantom/state")
        self.declare_parameter("output_pose_topic", "/geomagic_touch/pose")
        self.declare_parameter("output_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("frame_id", "geomagic_touch_base")
        self.declare_parameter("pose_position_scale", 0.001)
        self.declare_parameter("deadman_from_locked", True)
        self.declare_parameter("gripper_from_close_gripper", False)

        self.omni_state_topic = self.get_parameter("omni_state_topic").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.output_buttons_topic = self.get_parameter("output_buttons_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.pose_position_scale = float(self.get_parameter("pose_position_scale").value)
        self.deadman_from_locked = as_bool(self.get_parameter("deadman_from_locked").value)
        self.gripper_from_close_gripper = as_bool(
            self.get_parameter("gripper_from_close_gripper").value
        )

        try:
            from omni_msgs.msg import OmniState
        except ImportError as exc:
            raise RuntimeError(
                "omni_msgs is not available. Build/source the Geomagic Touch ROS 2 "
                "driver workspace, or publish PoseStamped/Joy directly."
            ) from exc

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.buttons_pub = self.create_publisher(Joy, self.output_buttons_topic, 10)
        self.create_subscription(OmniState, self.omni_state_topic, self.state_callback, 10)

        self.get_logger().info(
            f"Adapting {self.omni_state_topic} -> "
            f"{self.output_pose_topic}, {self.output_buttons_topic}"
        )

    def state_callback(self, msg) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.frame_id
        pose.pose = msg.pose
        pose.pose.position.x *= self.pose_position_scale
        pose.pose.position.y *= self.pose_position_scale
        pose.pose.position.z *= self.pose_position_scale
        self.pose_pub.publish(pose)

        buttons = Joy()
        buttons.header = msg.header
        deadman = bool(msg.locked) if self.deadman_from_locked else not bool(msg.locked)
        gripper = bool(msg.close_gripper) if self.gripper_from_close_gripper else False
        buttons.buttons = [1 if deadman else 0, 1 if gripper else 0]
        buttons.axes = []
        self.buttons_pub.publish(buttons)


def main():
    rclpy.init()
    try:
        node = GeomagicOmniStateAdapter()
    except RuntimeError as exc:
        node = Node("geomagic_omni_state_adapter_error")
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
