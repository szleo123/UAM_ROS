#!/usr/bin/env python3

import math
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


def _parse_csv(value, cast=str):
    if isinstance(value, (list, tuple)):
        return [cast(v) for v in value]
    text = str(value).strip()
    if text.startswith("["):
        text = text[1:]
    if text.endswith("]"):
        text = text[:-1]
    return [cast(item.strip()) for item in text.split(",") if item.strip()]


class DynamicsMonitor(Node):
    def __init__(self):
        super().__init__("arm_dynamics_monitor")

        self.declare_parameter("robot_description", "")
        self.declare_parameter("joint_names", "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_topic", "/arm_dynamics/torques_nm")
        self.declare_parameter("per_joint_topic_prefix", "/arm_dynamics")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("mode", "gravity")
        self.declare_parameter("torque_scale", 1.0)
        self.declare_parameter("torque_limits_nm", "0.5,0.5,0.3,0.2,0.15,0.1")
        self.declare_parameter("low_pass_alpha", 0.2)
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("plot_history_sec", 15.0)
        self.declare_parameter("plot_rate_hz", 10.0)

        self.joint_names = _parse_csv(self.get_parameter("joint_names").value, str)
        self.mode = str(self.get_parameter("mode").value).lower()
        self.torque_scale = float(self.get_parameter("torque_scale").value)
        self.torque_limits = np.array(
            _parse_csv(self.get_parameter("torque_limits_nm").value, float), dtype=float
        )
        if len(self.torque_limits) < len(self.joint_names):
            pad = np.full(len(self.joint_names) - len(self.torque_limits), self.torque_limits[-1])
            self.torque_limits = np.concatenate([self.torque_limits, pad])
        self.torque_limits = np.abs(self.torque_limits[: len(self.joint_names)])
        self.low_pass_alpha = min(max(float(self.get_parameter("low_pass_alpha").value), 0.0), 1.0)
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.last_tau = np.zeros(len(self.joint_names))
        self.last_q = np.zeros(len(self.joint_names))
        self.last_v = np.zeros(len(self.joint_names))
        self.have_history = False
        self.last_state_time = None
        self.latest_state = None
        self.state_lock = threading.Lock()

        self._load_pinocchio_model()

        output_topic = str(self.get_parameter("output_topic").value)
        self.torque_pub = self.create_publisher(Float64MultiArray, output_topic, 10)
        prefix = str(self.get_parameter("per_joint_topic_prefix").value).rstrip("/")
        self.joint_pubs = [
            self.create_publisher(Float64, f"{prefix}/{name}_torque_nm", 10)
            for name in self.joint_names
        ]
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            20,
        )

        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.plot_enabled = bool(self.get_parameter("enable_plot").value)
        self.plot_history_sec = max(float(self.get_parameter("plot_history_sec").value), 1.0)
        self.plot_rate_hz = max(float(self.get_parameter("plot_rate_hz").value), 1.0)
        self.plot_times = deque()
        self.plot_values = deque()
        self.plot_fig = None
        self.plot_ax = None
        self.plot_lines = []
        if self.plot_enabled:
            self._setup_plot()
            self.plot_timer = self.create_timer(1.0 / self.plot_rate_hz, self._update_plot)

        self.get_logger().warn(
            f"Dynamics monitor ready: mode={self.mode}, joints={self.joint_names}, "
            f"limits={self.torque_limits.tolist()}, plot={self.plot_enabled}"
        )

    def _load_pinocchio_model(self):
        try:
            import pinocchio as pin
        except Exception as exc:
            raise RuntimeError(
                "Python Pinocchio is required for dynamics_monitor.py. "
                "Install ros-humble-pinocchio or the matching Pinocchio Python package."
            ) from exc

        robot_description = str(self.get_parameter("robot_description").value)
        if not robot_description:
            raise RuntimeError("robot_description parameter is empty; cannot build Pinocchio model.")

        self.pin = pin
        self.model = pin.buildModelFromXML(robot_description)
        self.data = self.model.createData()
        self.q_indices = []
        self.v_indices = []
        for joint_name in self.joint_names:
            joint_id = self.model.getJointId(joint_name)
            if joint_id == 0 or joint_id >= len(self.model.joints):
                raise RuntimeError(f"Joint '{joint_name}' was not found in the Pinocchio model.")
            if self.model.nqs[joint_id] != 1 or self.model.nvs[joint_id] != 1:
                raise RuntimeError(
                    f"Joint '{joint_name}' is not 1-DoF "
                    f"(nq={self.model.nqs[joint_id]}, nv={self.model.nvs[joint_id]})."
                )
            self.q_indices.append(self.model.idx_qs[joint_id])
            self.v_indices.append(self.model.idx_vs[joint_id])

    def _setup_plot(self):
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().error(f"Matplotlib unavailable; torque plot disabled: {exc}")
            self.plot_enabled = False
            return

        self.plt = plt
        plt.ion()
        self.plot_fig, self.plot_ax = plt.subplots()
        self.plot_lines = [
            self.plot_ax.plot([], [], label=name)[0] for name in self.joint_names
        ]
        self.plot_ax.set_xlabel("time [s]")
        self.plot_ax.set_ylabel("torque [Nm]")
        self.plot_ax.grid(True)
        self.plot_ax.legend(loc="upper right")
        self.plot_fig.canvas.manager.set_window_title("Arm dynamics torque preview")
        self.plot_fig.show()

    def _on_joint_state(self, msg):
        positions = {}
        velocities = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                positions[name] = msg.position[i]
            if i < len(msg.velocity):
                velocities[name] = msg.velocity[i]

        if not all(name in positions for name in self.joint_names):
            return

        with self.state_lock:
            self.latest_state = (positions, velocities)
            self.last_state_time = self.get_clock().now()

    def _on_timer(self):
        with self.state_lock:
            state = self.latest_state
            state_time = self.last_state_time

        if state is None or state_time is None:
            return
        age = (self.get_clock().now() - state_time).nanoseconds * 1e-9
        if age > self.state_timeout_sec:
            self.get_logger().warn(
                f"Joint state is stale ({age:.3f}s); skipping dynamics update.",
                throttle_duration_sec=2.0,
            )
            return

        positions, velocities = state
        q = self.pin.neutral(self.model)
        v = np.zeros(self.model.nv)
        a = np.zeros(self.model.nv)

        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        dt = None
        if hasattr(self, "last_compute_stamp"):
            dt = max(stamp - self.last_compute_stamp, 1e-6)
        self.last_compute_stamp = stamp

        q_arm = np.array([float(positions[name]) for name in self.joint_names])
        if self.mode in ("coriolis", "full"):
            v_arm = np.array([float(velocities.get(name, 0.0)) for name in self.joint_names])
            if self.mode == "full" and dt is not None:
                a_arm = (v_arm - self.last_v) / dt
            else:
                a_arm = np.zeros(len(self.joint_names))
        else:
            v_arm = np.zeros(len(self.joint_names))
            a_arm = np.zeros(len(self.joint_names))

        for i, (q_index, v_index) in enumerate(zip(self.q_indices, self.v_indices)):
            q[q_index] = q_arm[i]
            v[v_index] = v_arm[i]
            a[v_index] = a_arm[i]

        try:
            tau_full = self.pin.rnea(self.model, self.data, q, v, a)
        except Exception as exc:
            self.get_logger().error(f"Pinocchio RNEA failed: {exc}", throttle_duration_sec=1.0)
            return

        tau = np.array([tau_full[v_index] for v_index in self.v_indices], dtype=float)
        tau *= self.torque_scale
        tau = np.clip(tau, -self.torque_limits, self.torque_limits)
        if self.low_pass_alpha <= 0.0:
            tau = self.last_tau
        elif self.low_pass_alpha < 1.0:
            tau = self.last_tau + self.low_pass_alpha * (tau - self.last_tau)

        self.last_tau = tau
        self.last_q = q_arm
        self.last_v = v_arm
        self._publish_tau(tau)
        self._record_plot_sample(stamp, tau)

    def _publish_tau(self, tau):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in tau]
        self.torque_pub.publish(msg)
        for pub, value in zip(self.joint_pubs, tau):
            scalar = Float64()
            scalar.data = float(value)
            pub.publish(scalar)

    def _record_plot_sample(self, stamp, tau):
        if not self.plot_enabled:
            return
        self.plot_times.append(stamp)
        self.plot_values.append(tau.copy())
        cutoff = stamp - self.plot_history_sec
        while self.plot_times and self.plot_times[0] < cutoff:
            self.plot_times.popleft()
            self.plot_values.popleft()

    def _update_plot(self):
        if not self.plot_enabled or not self.plot_times:
            return
        times = np.array(self.plot_times)
        values = np.vstack(self.plot_values)
        x = times - times[-1]
        for i, line in enumerate(self.plot_lines):
            line.set_data(x, values[:, i])
        self.plot_ax.set_xlim(-self.plot_history_sec, 0.0)
        limit = max(float(np.max(np.abs(values))) * 1.2, 0.1)
        self.plot_ax.set_ylim(-limit, limit)
        self.plot_fig.canvas.draw_idle()
        self.plot_fig.canvas.flush_events()


def main():
    rclpy.init()
    node = DynamicsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
