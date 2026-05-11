#!/usr/bin/env python3

import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String, UInt8
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ALL_READY = 4


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class TrajectoryDeadmanGate(Node):
    """Forward Servo trajectories only when armed, homed, deadman-held, and commanded."""

    def __init__(self):
        super().__init__("trajectory_deadman_gate")

        self.declare_parameter("input_trajectory_topic", "/arm_teleop/servo_raw_joint_trajectory")
        self.declare_parameter("output_trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("input_twist_topic", "/arm_teleop/twist_cmd")
        self.declare_parameter("input_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("homing_state_topic", "/arm_homing/state")
        self.declare_parameter("status_topic", "/arm_teleop/trajectory_gate_status")
        self.declare_parameter("deadman_button_index", 0)
        self.declare_parameter("arm_joints", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("require_deadman", True)
        self.declare_parameter("require_homing", True)
        self.declare_parameter("require_nonzero_twist", True)
        self.declare_parameter("button_timeout_s", 0.25)
        self.declare_parameter("twist_timeout_s", 0.25)
        self.declare_parameter("homing_timeout_s", 1.0)
        self.declare_parameter("min_linear_speed_m_s", 1.0e-5)
        self.declare_parameter("min_angular_speed_rad_s", 1.0e-4)
        self.declare_parameter("armed_on_start", False)
        self.declare_parameter("publish_hold_on_block", True)
        self.declare_parameter("hold_duration_s", 0.20)
        self.declare_parameter("integrate_servo_deltas", False)
        self.declare_parameter("integrated_command_scale", 1.0)
        self.declare_parameter("max_integrated_step_rad", 0.03)
        self.declare_parameter("integrated_command_duration_s", 0.05)
        self.declare_parameter("integrated_publish_rate_hz", 10.0)

        self.input_trajectory_topic = self.get_parameter("input_trajectory_topic").value
        self.output_trajectory_topic = self.get_parameter("output_trajectory_topic").value
        self.input_twist_topic = self.get_parameter("input_twist_topic").value
        self.input_buttons_topic = self.get_parameter("input_buttons_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.homing_state_topic = self.get_parameter("homing_state_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.arm_joints = list(self.get_parameter("arm_joints").value)
        self.require_deadman = as_bool(self.get_parameter("require_deadman").value)
        self.require_homing = as_bool(self.get_parameter("require_homing").value)
        self.require_nonzero_twist = as_bool(self.get_parameter("require_nonzero_twist").value)
        self.button_timeout_s = float(self.get_parameter("button_timeout_s").value)
        self.twist_timeout_s = float(self.get_parameter("twist_timeout_s").value)
        self.homing_timeout_s = float(self.get_parameter("homing_timeout_s").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed_m_s").value)
        self.min_angular_speed = float(self.get_parameter("min_angular_speed_rad_s").value)
        self.armed = as_bool(self.get_parameter("armed_on_start").value)
        self.publish_hold_on_block = as_bool(self.get_parameter("publish_hold_on_block").value)
        self.hold_duration_s = float(self.get_parameter("hold_duration_s").value)
        self.integrate_servo_deltas = as_bool(self.get_parameter("integrate_servo_deltas").value)
        self.integrated_command_scale = float(self.get_parameter("integrated_command_scale").value)
        self.max_integrated_step = abs(float(self.get_parameter("max_integrated_step_rad").value))
        self.integrated_command_duration_s = float(
            self.get_parameter("integrated_command_duration_s").value
        )
        self.integrated_publish_period_s = 1.0 / max(
            float(self.get_parameter("integrated_publish_rate_hz").value), 1.0
        )

        self.buttons: List[int] = []
        self.latest_button_time = self.get_clock().now()
        self.latest_twist: Optional[TwistStamped] = None
        self.latest_twist_time = self.get_clock().now()
        self.homing_state: Optional[int] = None
        self.latest_homing_time = self.get_clock().now()
        self.latest_joint_positions: Dict[str, float] = {}
        self.integrated_positions: Dict[str, float] = {}
        self.last_integrated_publish_time = self.get_clock().now()
        self.forwarding_active = False
        self.forwarded_count = 0
        self.blocked_count = 0
        self.hold_count = 0
        self.last_reason = "startup_disarmed"

        self.trajectory_pub = self.create_publisher(JointTrajectory, self.output_trajectory_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(JointTrajectory, self.input_trajectory_topic, self.trajectory_callback, 10)
        self.create_subscription(TwistStamped, self.input_twist_topic, self.twist_callback, 10)
        self.create_subscription(Joy, self.input_buttons_topic, self.buttons_callback, 10)
        self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 20)
        self.create_subscription(UInt8, self.homing_state_topic, self.homing_callback, 10)
        self.create_service(SetBool, "~/set_armed", self.set_armed_callback)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().warn(
            f"Trajectory gate active: armed={self.armed}, homing_required={self.require_homing}, "
            f"input={self.input_trajectory_topic}, output={self.output_trajectory_topic}, "
            f"integrate_servo_deltas={self.integrate_servo_deltas}"
        )

    def buttons_callback(self, msg: Joy) -> None:
        self.buttons = [1 if value else 0 for value in msg.buttons]
        self.latest_button_time = self.get_clock().now()
        self.check_falling_edge()

    def twist_callback(self, msg: TwistStamped) -> None:
        self.latest_twist = msg
        self.latest_twist_time = self.get_clock().now()
        self.check_falling_edge()

    def homing_callback(self, msg: UInt8) -> None:
        self.homing_state = int(msg.data)
        self.latest_homing_time = self.get_clock().now()
        self.check_falling_edge()

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.arm_joints:
                self.latest_joint_positions[name] = float(position)

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

    def command_active(self) -> bool:
        if not self.require_nonzero_twist:
            return True
        if self.latest_twist is None:
            return False
        fresh = (self.get_clock().now() - self.latest_twist_time) < Duration(
            seconds=self.twist_timeout_s
        )
        if not fresh:
            return False
        linear = self.latest_twist.twist.linear
        angular = self.latest_twist.twist.angular
        linear_norm = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        angular_norm = math.sqrt(
            angular.x * angular.x + angular.y * angular.y + angular.z * angular.z
        )
        return linear_norm > self.min_linear_speed or angular_norm > self.min_angular_speed

    def allowed(self) -> tuple[bool, str]:
        if not self.armed:
            return False, "disarmed"
        if not self.homing_ready():
            return False, "homing_not_ready"
        if not self.deadman_active():
            return False, "deadman_released"
        if not self.command_active():
            return False, "no_active_command"
        return True, "allowed"

    def trajectory_callback(self, msg: JointTrajectory) -> None:
        is_allowed, reason = self.allowed()
        self.last_reason = reason
        if is_allowed:
            output = self.integrate_trajectory(msg) if self.integrate_servo_deltas else msg
            if output is None:
                if self.integrate_servo_deltas:
                    self.forwarding_active = True
                    self.forwarded_count += 1
                else:
                    self.blocked_count += 1
                    self.last_reason = "waiting_for_joint_state"
                return
            self.trajectory_pub.publish(output)
            self.forwarded_count += 1
            self.forwarding_active = True
        else:
            self.blocked_count += 1
            if self.forwarding_active:
                self.publish_hold()
                self.forwarding_active = False
            self.integrated_positions = {}

    def integrate_trajectory(self, msg: JointTrajectory) -> Optional[JointTrajectory]:
        if not msg.points or not msg.points[0].positions:
            return None
        if any(joint not in self.latest_joint_positions for joint in self.arm_joints):
            return None
        if any(joint not in self.integrated_positions for joint in self.arm_joints):
            self.integrated_positions = {
                joint: self.latest_joint_positions[joint] for joint in self.arm_joints
            }

        target_by_joint = {
            joint: float(position)
            for joint, position in zip(msg.joint_names, msg.points[0].positions)
            if joint in self.arm_joints
        }
        if any(joint not in target_by_joint for joint in self.arm_joints):
            return None

        output = JointTrajectory()
        output.header.stamp = self.get_clock().now().to_msg()
        output.joint_names = list(self.arm_joints)
        point = JointTrajectoryPoint()
        for joint in self.arm_joints:
            servo_step = target_by_joint[joint] - self.latest_joint_positions[joint]
            servo_step = clamp(
                servo_step * self.integrated_command_scale,
                self.max_integrated_step,
            )
            self.integrated_positions[joint] += servo_step
            point.positions.append(self.integrated_positions[joint])

        now = self.get_clock().now()
        elapsed = (now - self.last_integrated_publish_time).nanoseconds * 1e-9
        if elapsed < self.integrated_publish_period_s:
            return None
        self.last_integrated_publish_time = now

        point.time_from_start.sec = int(self.integrated_command_duration_s)
        point.time_from_start.nanosec = int(
            (self.integrated_command_duration_s % 1.0) * 1e9
        )
        output.points = [point]
        return output

    def check_falling_edge(self) -> None:
        is_allowed, reason = self.allowed()
        self.last_reason = reason
        if self.forwarding_active and not is_allowed:
            self.publish_hold()
            self.forwarding_active = False
            self.integrated_positions = {}

    def publish_hold(self) -> None:
        if not self.publish_hold_on_block:
            return
        if any(joint not in self.latest_joint_positions for joint in self.arm_joints):
            self.get_logger().warn("Cannot publish hold: waiting for all arm joint states.")
            return
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.arm_joints)
        point = JointTrajectoryPoint()
        point.positions = [self.latest_joint_positions[joint] for joint in self.arm_joints]
        point.time_from_start.sec = int(self.hold_duration_s)
        point.time_from_start.nanosec = int((self.hold_duration_s % 1.0) * 1e9)
        msg.points = [point]
        self.trajectory_pub.publish(msg)
        self.hold_count += 1
        self.integrated_positions = {}
        self.get_logger().warn("Published hold trajectory after teleop gate blocked motion.")

    def set_armed_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.armed = bool(request.data)
        if not self.armed and self.forwarding_active:
            self.publish_hold()
            self.forwarding_active = False
        if not self.armed:
            self.integrated_positions = {}
        response.success = True
        response.message = f"trajectory gate {'armed' if self.armed else 'disarmed'}"
        self.publish_status()
        return response

    def publish_status(self) -> None:
        msg = String()
        msg.data = (
            f"armed={int(self.armed)}; homed={int(self.homing_ready())}; "
            f"homing_state={self.homing_state}; deadman={int(self.deadman_active())}; "
            f"command={int(self.command_active())}; reason={self.last_reason}; "
            f"forwarded={self.forwarded_count}; blocked={self.blocked_count}; holds={self.hold_count}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryDeadmanGate()
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
