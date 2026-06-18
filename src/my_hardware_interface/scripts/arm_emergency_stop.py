#!/usr/bin/env python3

from action_msgs.srv import CancelGoal
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class ArmEmergencyStop(Node):
    """Cancel active arm trajectory goals and explicitly hold current joint positions."""

    def __init__(self):
        super().__init__("arm_emergency_stop")

        self.declare_parameter("service_name", "/arm_emergency_stop/trigger")
        self.declare_parameter(
            "follow_joint_trajectory_action_name",
            "/arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("joint_trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_names", "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6")
        self.declare_parameter("hold_duration_s", 0.2)
        self.declare_parameter("joint_state_timeout_s", 0.5)
        self.declare_parameter("cancel_before_hold", True)
        self.declare_parameter("publish_hold", True)

        self.service_name = self.get_parameter("service_name").value
        self.action_name = self.get_parameter("follow_joint_trajectory_action_name").value
        self.trajectory_topic = self.get_parameter("joint_trajectory_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.joint_names = self._parse_string_list(self.get_parameter("joint_names").value)
        self.hold_duration_s = float(self.get_parameter("hold_duration_s").value)
        self.joint_state_timeout_s = float(self.get_parameter("joint_state_timeout_s").value)
        self.cancel_before_hold = as_bool(self.get_parameter("cancel_before_hold").value)
        self.publish_hold_enabled = as_bool(self.get_parameter("publish_hold").value)

        self.latest_joint_positions = {}
        self.latest_joint_state_time = None

        self.cancel_client = self.create_client(
            CancelGoal, self._action_cancel_service_name(self.action_name)
        )
        self.hold_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            QoSProfile(depth=20),
        )
        self.create_service(Trigger, self.service_name, self.trigger_callback)

        self.get_logger().warn(
            "Arm emergency stop ready: "
            f"service={self.service_name}, action={self.action_name}, "
            f"hold_topic={self.trajectory_topic}"
        )

    @staticmethod
    def _action_cancel_service_name(action_name):
        return f"{action_name.rstrip('/')}/_action/cancel_goal"

    @staticmethod
    def _parse_string_list(value):
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return [str(item) for item in value]

    def joint_state_callback(self, msg):
        now = self.get_clock().now()
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.latest_joint_positions[name] = float(position)
        self.latest_joint_state_time = now

    def trigger_callback(self, request, response):
        del request

        cancel_requested = self.request_cancel()
        hold_published = self.publish_hold()

        response.success = cancel_requested or hold_published
        response.message = (
            "Arm emergency stop: "
            f"cancel_requested={int(cancel_requested)}, "
            f"hold_published={int(hold_published)}"
        )
        self.get_logger().warn(response.message)
        return response

    def request_cancel(self):
        if not self.cancel_before_hold:
            return False
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn(
                "Arm trajectory cancel service is not available: "
                f"{self._action_cancel_service_name(self.action_name)}"
            )
            return False

        request = CancelGoal.Request()
        future = self.cancel_client.call_async(request)
        future.add_done_callback(self.cancel_done_callback)
        return True

    def cancel_done_callback(self, future):
        try:
            result = future.result()
            self.get_logger().warn(
                "Arm trajectory cancel response: return_code=%d, goals_canceling=%d",
                result.return_code,
                len(result.goals_canceling),
            )
        except Exception as exc:
            self.get_logger().warn(f"Arm trajectory cancel request failed: {exc}")

    def publish_hold(self):
        if not self.publish_hold_enabled:
            return False
        if not self.latest_joint_state_time:
            self.get_logger().warn("Cannot publish arm hold: no joint state received yet.")
            return False
        if self.get_clock().now() - self.latest_joint_state_time > Duration(
            seconds=self.joint_state_timeout_s
        ):
            self.get_logger().warn("Cannot publish arm hold: joint state is stale.")
            return False
        if any(joint not in self.latest_joint_positions for joint in self.joint_names):
            self.get_logger().warn("Cannot publish arm hold: missing one or more arm joints.")
            return False

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = [self.latest_joint_positions[joint] for joint in self.joint_names]
        point.time_from_start = Duration(seconds=self.hold_duration_s).to_msg()
        msg.points = [point]

        self.hold_pub.publish(msg)
        self.get_logger().warn("Published arm hold-current-position trajectory.")
        return True


def main():
    rclpy.init()
    node = ArmEmergencyStop()
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
