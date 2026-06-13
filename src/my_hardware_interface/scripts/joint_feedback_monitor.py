#!/usr/bin/env python3

import csv
import math
import threading
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _parse_csv(value, cast=str):
    if isinstance(value, (list, tuple)):
        return [cast(v) for v in value]
    text = str(value).strip()
    if text.startswith("["):
        text = text[1:]
    if text.endswith("]"):
        text = text[:-1]
    return [cast(item.strip()) for item in text.split(",") if item.strip()]


def _finite_or_nan(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class JointFeedbackMonitor(Node):
    def __init__(self):
        super().__init__("joint_feedback_monitor")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("joint_names", "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("controller_state_topic", "/arm_controller/controller_state")
        self.declare_parameter("raw_velocity_topic", "/my_arm_system/raw_feedback_velocities_rad_s")
        self.declare_parameter("torque_reference_topic", "/my_arm_system/dynamics_tau_ff_nm")
        self.declare_parameter("output_error_topic", "/arm_joint_feedback/error_rad")
        self.declare_parameter("rate_hz", 50.0, dynamic_param)
        self.declare_parameter("state_timeout_sec", 0.5, dynamic_param)
        self.declare_parameter("plot", True, dynamic_param)
        self.declare_parameter("plot_history_sec", 15.0, dynamic_param)
        self.declare_parameter("plot_rate_hz", 10.0, dynamic_param)
        self.declare_parameter("plot_min_rad", -1.0, dynamic_param)
        self.declare_parameter("plot_max_rad", 1.0, dynamic_param)
        self.declare_parameter("plot_velocity_abs", 2.0, dynamic_param)
        self.declare_parameter("plot_raw_velocity", False, dynamic_param)
        self.declare_parameter("plot_torque_abs", 5.0, dynamic_param)
        self.declare_parameter("csv_enabled", False, dynamic_param)
        self.declare_parameter("csv_file", "/tmp/joint_monitor.csv")
        self.declare_parameter("csv_append", False, dynamic_param)

        self.joint_names = _parse_csv(self.get_parameter("joint_names").value, str)
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.plot_enabled = _as_bool(self.get_parameter("plot").value)
        self.plot_history_sec = max(float(self.get_parameter("plot_history_sec").value), 1.0)
        self.plot_rate_hz = max(float(self.get_parameter("plot_rate_hz").value), 1.0)
        self.plot_min_rad = float(self.get_parameter("plot_min_rad").value)
        self.plot_max_rad = float(self.get_parameter("plot_max_rad").value)
        if self.plot_min_rad > self.plot_max_rad:
            self.plot_min_rad, self.plot_max_rad = self.plot_max_rad, self.plot_min_rad
        self.plot_velocity_abs = abs(float(self.get_parameter("plot_velocity_abs").value))
        self.plot_raw_velocity = _as_bool(self.get_parameter("plot_raw_velocity").value)
        self.plot_torque_abs = abs(float(self.get_parameter("plot_torque_abs").value))
        self.csv_enabled = _as_bool(self.get_parameter("csv_enabled").value)
        self.csv_path = Path(str(self.get_parameter("csv_file").value))
        self.csv_append = _as_bool(self.get_parameter("csv_append").value)

        self.state_lock = threading.Lock()
        self.latest = {
            "position_feedback": None,
            "position_reference": None,
            "velocity_feedback": None,
            "velocity_feedback_raw": None,
            "velocity_reference": None,
            "torque_feedback": None,
            "torque_reference": None,
        }
        self.latest_state_time = None
        self.latest_raw_velocity_time = None
        self.latest_reference_time = None
        self.latest_torque_reference_time = None

        self.times = deque()
        self.history = {key: deque() for key in self.latest}
        self.history["position_error"] = deque()
        self.history["velocity_error"] = deque()
        self.history["torque_error"] = deque()

        self.error_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("output_error_topic").value), 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            20,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            str(self.get_parameter("controller_state_topic").value),
            self._on_controller_state,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("raw_velocity_topic").value),
            self._on_raw_velocity,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("torque_reference_topic").value),
            self._on_torque_reference,
            10,
        )

        if self.csv_enabled:
            self._open_csv()
        else:
            self.csv_file = None
            self.csv_writer = None

        if self.plot_enabled:
            self._setup_plot()

        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self.sample_timer = self.create_timer(1.0 / rate_hz, self._sample)
        if self.plot_enabled:
            self.plot_timer = self.create_timer(1.0 / self.plot_rate_hz, self._update_plot)

        self.get_logger().warn(
            f"Joint feedback monitor ready: joints={self.joint_names}, "
            f"plot={self.plot_enabled}, raw_velocity={self.plot_raw_velocity}, csv={self.csv_enabled}"
        )

    def _open_csv(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "a" if self.csv_append else "w"
        file_exists = self.csv_path.exists() and self.csv_path.stat().st_size > 0
        self.csv_file = self.csv_path.open(mode, newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if not self.csv_append or not file_exists:
            self.csv_writer.writerow(
                [
                    "timestamp_sec",
                    "joint_name",
                    "position_feedback_rad",
                    "position_reference_rad",
                    "position_error_rad",
                    "velocity_feedback_rad_s",
                    "velocity_feedback_raw_rad_s",
                    "velocity_reference_rad_s",
                    "velocity_error_rad_s",
                    "torque_feedback_nm",
                    "torque_reference_nm",
                    "torque_error_nm",
                ]
            )
            self.csv_file.flush()

    def _setup_plot(self):
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().error(f"Matplotlib unavailable; joint feedback plot disabled: {exc}")
            self.plot_enabled = False
            return

        self.plt = plt
        plt.ion()
        rows = len(self.joint_names)
        self.fig, axes = plt.subplots(
            rows, 3, sharex=True, figsize=(14, max(6, 1.8 * rows)), squeeze=False
        )
        self.axes = axes
        self.lines = {}
        columns = [
            ("position", "position [rad]", (self.plot_min_rad, self.plot_max_rad)),
            ("velocity", "velocity [rad/s]", (-self.plot_velocity_abs, self.plot_velocity_abs)),
            ("torque", "torque [Nm]", (-self.plot_torque_abs, self.plot_torque_abs)),
        ]

        for row, name in enumerate(self.joint_names):
            for col, (kind, ylabel, ylim) in enumerate(columns):
                ax = self.axes[row][col]
                feedback_line, = ax.plot([], [], label="feedback", color="tab:blue")
                reference_line, = ax.plot([], [], label="reference", color="tab:orange")
                error_line, = ax.plot([], [], label="error", color="tab:red", alpha=0.45)
                self.lines[(kind, row, "feedback")] = feedback_line
                self.lines[(kind, row, "reference")] = reference_line
                self.lines[(kind, row, "error")] = error_line
                if kind == "velocity":
                    raw_line, = ax.plot(
                        [],
                        [],
                        label="raw feedback",
                        color="tab:purple",
                        linestyle="--",
                        alpha=0.75,
                    )
                    self.lines[(kind, row, "raw_feedback")] = raw_line
                ax.grid(True)
                ax.set_ylim(*ylim)
                if row == 0:
                    ax.set_title(kind)
                if col == 0:
                    ax.set_ylabel(name)
                if col == 2:
                    ax.yaxis.set_label_position("right")
                    ax.set_ylabel(ylabel)
        self.axes[-1][0].set_xlabel("time [s]")
        self.axes[-1][1].set_xlabel("time [s]")
        self.axes[-1][2].set_xlabel("time [s]")
        self.axes[0][0].legend(loc="upper right")
        self.axes[0][1].legend(loc="upper right")
        self.fig.canvas.manager.set_window_title("Arm joint feedback monitor")
        self.fig.tight_layout()
        self.fig.show()

    def _array_from_joint_state(self, msg, field_name):
        values = getattr(msg, field_name)
        lookup = {}
        for i, name in enumerate(msg.name):
            if i < len(values):
                lookup[name] = values[i]
        if not all(name in lookup for name in self.joint_names):
            return None
        return np.array([_finite_or_nan(lookup[name]) for name in self.joint_names], dtype=float)

    def _on_joint_state(self, msg):
        position = self._array_from_joint_state(msg, "position")
        if position is None:
            return
        velocity = self._array_from_joint_state(msg, "velocity")
        effort = self._array_from_joint_state(msg, "effort")
        now = self.get_clock().now()
        with self.state_lock:
            self.latest["position_feedback"] = position
            self.latest["velocity_feedback"] = velocity
            self.latest["torque_feedback"] = effort
            self.latest_state_time = now

    def _array_from_controller_field(self, joint_names, values):
        lookup = {}
        for i, name in enumerate(joint_names):
            if i < len(values):
                lookup[name] = values[i]
        if not all(name in lookup for name in self.joint_names):
            return None
        return np.array([_finite_or_nan(lookup[name]) for name in self.joint_names], dtype=float)

    def _on_controller_state(self, msg):
        if not msg.joint_names:
            return
        ref_pos = self._array_from_controller_field(msg.joint_names, msg.reference.positions)
        fb_pos = self._array_from_controller_field(msg.joint_names, msg.feedback.positions)
        ref_vel = self._array_from_controller_field(msg.joint_names, msg.reference.velocities)
        fb_vel = self._array_from_controller_field(msg.joint_names, msg.feedback.velocities)

        now = self.get_clock().now()
        with self.state_lock:
            if ref_pos is not None:
                self.latest["position_reference"] = ref_pos
                self.latest_reference_time = now
            if ref_vel is not None:
                self.latest["velocity_reference"] = ref_vel
                self.latest_reference_time = now
            if fb_pos is not None:
                self.latest["position_feedback"] = fb_pos
                self.latest_state_time = now
            if fb_vel is not None:
                self.latest["velocity_feedback"] = fb_vel
                self.latest_state_time = now

    def _on_torque_reference(self, msg):
        if len(msg.data) < len(self.joint_names):
            return
        reference = np.array(
            [_finite_or_nan(value) for value in msg.data[: len(self.joint_names)]],
            dtype=float,
        )
        with self.state_lock:
            self.latest["torque_reference"] = reference
            self.latest_torque_reference_time = self.get_clock().now()

    def _on_raw_velocity(self, msg):
        if len(msg.data) < len(self.joint_names):
            return
        raw_velocity = np.array(
            [_finite_or_nan(value) for value in msg.data[: len(self.joint_names)]],
            dtype=float,
        )
        with self.state_lock:
            self.latest["velocity_feedback_raw"] = raw_velocity
            self.latest_raw_velocity_time = self.get_clock().now()

    def _fresh_or_none(self, value, timestamp, now):
        if value is None or timestamp is None:
            return None
        age = (now - timestamp).nanoseconds * 1e-9
        return value.copy() if age <= self.state_timeout_sec else None

    def _sample(self):
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        with self.state_lock:
            latest = {key: None if value is None else value.copy() for key, value in self.latest.items()}
            state_time = self.latest_state_time
            raw_velocity_time = self.latest_raw_velocity_time
            reference_time = self.latest_reference_time
            torque_reference_time = self.latest_torque_reference_time

        position_feedback = self._fresh_or_none(latest["position_feedback"], state_time, now)
        if position_feedback is None:
            return

        position_reference = self._fresh_or_none(latest["position_reference"], reference_time, now)
        velocity_feedback = self._fresh_or_none(latest["velocity_feedback"], state_time, now)
        velocity_feedback_raw = self._fresh_or_none(
            latest["velocity_feedback_raw"], raw_velocity_time, now
        )
        velocity_reference = self._fresh_or_none(latest["velocity_reference"], reference_time, now)
        torque_feedback = self._fresh_or_none(latest["torque_feedback"], state_time, now)
        torque_reference = self._fresh_or_none(latest["torque_reference"], torque_reference_time, now)

        zero = np.zeros(len(self.joint_names), dtype=float)
        position_reference = position_feedback.copy() if position_reference is None else position_reference
        velocity_feedback = zero.copy() if velocity_feedback is None else velocity_feedback
        velocity_feedback_raw = (
            velocity_feedback.copy() if velocity_feedback_raw is None else velocity_feedback_raw
        )
        velocity_reference = velocity_feedback.copy() if velocity_reference is None else velocity_reference
        torque_feedback = zero.copy() if torque_feedback is None else torque_feedback
        torque_reference = zero.copy() if torque_reference is None else torque_reference

        position_error = position_reference - position_feedback
        velocity_error = velocity_reference - velocity_feedback
        torque_error = torque_reference - torque_feedback

        self._publish_error(position_error)
        self._record_sample(
            stamp,
            position_feedback,
            position_reference,
            position_error,
            velocity_feedback,
            velocity_feedback_raw,
            velocity_reference,
            velocity_error,
            torque_feedback,
            torque_reference,
            torque_error,
        )

    def _publish_error(self, error):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in error]
        self.error_pub.publish(msg)

    def _record_sample(
        self,
        stamp,
        position_feedback,
        position_reference,
        position_error,
        velocity_feedback,
        velocity_feedback_raw,
        velocity_reference,
        velocity_error,
        torque_feedback,
        torque_reference,
        torque_error,
    ):
        samples = {
            "position_feedback": position_feedback,
            "position_reference": position_reference,
            "position_error": position_error,
            "velocity_feedback": velocity_feedback,
            "velocity_feedback_raw": velocity_feedback_raw,
            "velocity_reference": velocity_reference,
            "velocity_error": velocity_error,
            "torque_feedback": torque_feedback,
            "torque_reference": torque_reference,
            "torque_error": torque_error,
        }
        self.times.append(stamp)
        for key, value in samples.items():
            self.history[key].append(value.copy())
        cutoff = stamp - self.plot_history_sec
        while self.times and self.times[0] < cutoff:
            self.times.popleft()
            for history in self.history.values():
                history.popleft()

        if self.csv_writer:
            for i, name in enumerate(self.joint_names):
                self.csv_writer.writerow(
                    [
                        f"{stamp:.6f}",
                        name,
                        f"{position_feedback[i]:.6f}",
                        f"{position_reference[i]:.6f}",
                        f"{position_error[i]:.6f}",
                        f"{velocity_feedback[i]:.6f}",
                        f"{velocity_feedback_raw[i]:.6f}",
                        f"{velocity_reference[i]:.6f}",
                        f"{velocity_error[i]:.6f}",
                        f"{torque_feedback[i]:.6f}",
                        f"{torque_reference[i]:.6f}",
                        f"{torque_error[i]:.6f}",
                    ]
                )
            self.csv_file.flush()

    def _stack_history(self, key):
        return np.vstack(self.history[key]) if self.history[key] else None

    def _set_lines(self, kind, row, x, feedback, reference, error, raw_feedback=None):
        self.lines[(kind, row, "feedback")].set_data(x, feedback[:, row])
        self.lines[(kind, row, "reference")].set_data(x, reference[:, row])
        self.lines[(kind, row, "error")].set_data(x, error[:, row])
        if kind == "velocity" and (kind, row, "raw_feedback") in self.lines:
            raw_line = self.lines[(kind, row, "raw_feedback")]
            if self.plot_raw_velocity and raw_feedback is not None:
                raw_line.set_data(x, raw_feedback[:, row])
            else:
                raw_line.set_data([], [])

    def _update_axis_limits(self, ax, values, fallback):
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            ax.set_ylim(*fallback)
            return
        lo = min(fallback[0], float(np.min(finite)) - 0.05)
        hi = max(fallback[1], float(np.max(finite)) + 0.05)
        if hi - lo < 1e-3:
            hi += 0.1
            lo -= 0.1
        ax.set_ylim(lo, hi)

    def _update_plot(self):
        if not self.plot_enabled or not self.times:
            return
        times = np.array(self.times)
        x = times - times[-1]
        data = {key: self._stack_history(key) for key in self.history}
        if any(value is None for value in data.values()):
            return

        fallbacks = {
            "position": (self.plot_min_rad, self.plot_max_rad),
            "velocity": (-self.plot_velocity_abs, self.plot_velocity_abs),
            "torque": (-self.plot_torque_abs, self.plot_torque_abs),
        }
        for row in range(len(self.joint_names)):
            for col, kind in enumerate(("position", "velocity", "torque")):
                feedback = data[f"{kind}_feedback"]
                reference = data[f"{kind}_reference"]
                error = data[f"{kind}_error"]
                raw_feedback = data["velocity_feedback_raw"] if kind == "velocity" else None
                self._set_lines(kind, row, x, feedback, reference, error, raw_feedback)
                ax = self.axes[row][col]
                ax.set_xlim(-self.plot_history_sec, 0.0)
                values = np.concatenate([feedback[:, row], reference[:, row], error[:, row]])
                if kind == "velocity" and self.plot_raw_velocity and raw_feedback is not None:
                    values = np.concatenate([values, raw_feedback[:, row]])
                self._update_axis_limits(ax, values, fallbacks[kind])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def destroy_node(self):
        if getattr(self, "csv_file", None):
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = JointFeedbackMonitor()
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
