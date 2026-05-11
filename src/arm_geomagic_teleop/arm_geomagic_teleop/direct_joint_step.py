#!/usr/bin/env python3

from typing import Dict, List

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DirectJointStep(Node):
    """Publish one explicit JointTrajectory point from the current joint state."""

    def __init__(self):
        super().__init__("direct_joint_step")

        self.declare_parameter("joint_trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_name", "joint_1")
        self.declare_parameter("delta_rad", 0.10)
        self.declare_parameter("duration_s", 2.0)
        self.declare_parameter(
            "arm_joints",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )

        self.joint_trajectory_topic = self.get_parameter("joint_trajectory_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.joint_name = self.get_parameter("joint_name").value
        self.delta_rad = float(self.get_parameter("delta_rad").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.arm_joints = list(self.get_parameter("arm_joints").value)

        if self.joint_name not in self.arm_joints:
            raise ValueError(f"{self.joint_name} is not in arm_joints")

        self.latest_positions: Dict[str, float] = {}
        self.published = False

        self.pub = self.create_publisher(JointTrajectory, self.joint_trajectory_topic, 10)
        self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().warn(
            f"Waiting for joint states, then publishing {self.joint_name} += "
            f"{self.delta_rad:.3f} rad over {self.duration_s:.2f}s to {self.joint_trajectory_topic}"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.arm_joints:
                self.latest_positions[name] = float(position)

    def timer_callback(self) -> None:
        if self.published:
            return
        if any(joint not in self.latest_positions for joint in self.arm_joints):
            return

        positions: List[float] = [self.latest_positions[joint] for joint in self.arm_joints]
        positions[self.arm_joints.index(self.joint_name)] += self.delta_rad

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=self.duration_s).to_msg()

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.arm_joints)
        msg.points = [point]

        self.pub.publish(msg)
        self.published = True
        self.get_logger().warn("Published direct joint step.")


def main():
    rclpy.init()
    try:
        node = DirectJointStep()
    except ValueError as exc:
        node = Node("direct_joint_step_error")
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
