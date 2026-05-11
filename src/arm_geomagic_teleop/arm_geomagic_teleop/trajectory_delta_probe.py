#!/usr/bin/env python3

import math
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from trajectory_msgs.msg import JointTrajectory


def point_positions(msg: JointTrajectory) -> Optional[List[float]]:
    if not msg.points or not msg.points[0].positions:
        return None
    return [float(value) for value in msg.points[0].positions]


def vector_delta_norm(previous: List[float], current: List[float]) -> float:
    count = min(len(previous), len(current))
    if count == 0:
        return 0.0
    return math.sqrt(sum((current[index] - previous[index]) ** 2 for index in range(count)))


class TrajectoryDeltaProbe(Node):
    """Report how much incoming trajectory point positions change over time."""

    def __init__(self):
        super().__init__("trajectory_delta_probe")
        self.declare_parameter("topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("sample_count", 100)

        self.topic = self.get_parameter("topic").value
        self.sample_count = int(self.get_parameter("sample_count").value)
        self.previous: Optional[List[float]] = None
        self.count = 0
        self.max_delta = 0.0
        self.sum_delta = 0.0
        self.first: Optional[List[float]] = None
        self.last: Optional[List[float]] = None

        self.create_subscription(JointTrajectory, self.topic, self.callback, 20)
        self.get_logger().warn(
            f"Measuring {self.sample_count} trajectory deltas on {self.topic}. "
            "Hold the deadman and move while this runs."
        )

    def callback(self, msg: JointTrajectory) -> None:
        current = point_positions(msg)
        if current is None:
            return
        if self.first is None:
            self.first = list(current)
        if self.previous is not None:
            delta = vector_delta_norm(self.previous, current)
            self.max_delta = max(self.max_delta, delta)
            self.sum_delta += delta
            self.count += 1
        self.previous = list(current)
        self.last = list(current)

        if self.count >= self.sample_count:
            avg_delta = self.sum_delta / max(self.count, 1)
            total_delta = vector_delta_norm(self.first or [], self.last or [])
            self.get_logger().warn(
                f"Trajectory delta probe complete: samples={self.count}, "
                f"avg_step_delta={avg_delta:.6f} rad, max_step_delta={self.max_delta:.6f} rad, "
                f"first_to_last_delta={total_delta:.6f} rad"
            )
            rclpy.shutdown()


def main():
    rclpy.init()
    node = TrajectoryDeltaProbe()
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
