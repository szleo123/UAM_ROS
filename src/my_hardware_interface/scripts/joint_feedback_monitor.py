#!/usr/bin/env python3

import csv
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


class JointFeedbackMonitor(Node):
    def __init__(self):
        super().__init__("joint_feedback_monitor")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("joint_names", "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("controller_state_topic", "/arm_controller/controller_state")
        self.declare_parameter("output_error_topic", "/arm_joint_feedback/error_rad")
        self.declare_parameter("rate_hz", 50.0, dynamic_param)
        self.declare_parameter("state_timeout_sec", 0.5, dynamic_param)
        self.declare_parameter("plot", True, dynamic_param)
        self.declare_parameter("plot_history_sec", 15.0, dynamic_param)
        self.declare_parameter("plot_rate_hz", 10.0, dynamic_param)
        self.declare_parameter("plot_min_rad", -1.0, dynamic_param)
        self.declare_parameter("plot_max_rad", 1.0, dynamic_param)
        self.declare_parameter("csv_enabled", False, dynamic_param)
        self.declare_parameter("csv_file", "/tmp/joint_monitor.csv")
        self.declare_parameter("csv_append", False, dynamic_param)

        self.joint_names = _parse_csv(self.get_parameter("joint_names").value, str)
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.plot_enabled = bool(self.get_parameter("plot").value)
        self.plot_history_sec = max(float(self.get_parameter("plot_history_sec").value), 1.0)
        self.plot_rate_hz = max(float(self.get_parameter("plot_rate_hz").value), 1.0)
        self.plot_min_rad = float(self.get_parameter("plot_min_rad").value)
        self.plot_max_rad = float(self.get_parameter("plot_max_rad").value)
        if self.plot_min_rad > self.plot_max_rad:
            self.plot_min_rad, self.plot_max_rad = self.plot_max_rad, self.plot_min_rad
        self.csv_enabled = bool(self.get_parameter("csv_enabled").value)
        self.csv_path = Path(str(self.get_parameter("csv_file").value))
        self.csv_append = bool(self.get_parameter("csv_append").value)

        self.state_lock = threading.Lock()
        self.latest_feedback = None
        self.latest_reference = None
        self.latest_state_time = None
        self.latest_reference_time = None

        self.times = deque()
        self.feedback_history = deque()
        self.reference_history = deque()
        self.error_history = deque()

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
            f"plot={self.plot_enabled}, csv={self.csv_enabled}"
        )

    def _open_csv(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "a" if self.csv_append else "w"
        file_exists = self.csv_path.exists() and self.csv_path.stat().st_size > 0
        self.csv_file = self.csv_path.open(mode, newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if not self.csv_append or not file_exists:
            self.csv_writer.writerow(
                ["timestamp_sec", "joint_name", "feedback_rad", "reference_rad", "error_rad"]
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
        self.fig, axes = plt.subplots(rows, 1, sharex=True, figsize=(9, max(5, 1.8 * rows)))
        self.axes = np.atleast_1d(axes)
        self.feedback_lines = []
        self.reference_lines = []
        self.error_lines = []
        for ax, name in zip(self.axes, self.joint_names):
            feedback_line, = ax.plot([], [], label="feedback", color="tab:blue")
            reference_line, = ax.plot([], [], label="reference", color="tab:orange")
            error_line, = ax.plot([], [], label="error", color="tab:red", alpha=0.45)
            ax.set_ylabel(name)
            ax.grid(True)
            ax.set_ylim(self.plot_min_rad, self.plot_max_rad)
            self.feedback_lines.append(feedback_line)
            self.reference_lines.append(reference_line)
            self.error_lines.append(error_line)
        self.axes[-1].set_xlabel("time [s]")
        self.axes[0].legend(loc="upper right")
        self.fig.canvas.manager.set_window_title("Arm joint feedback monitor")
        self.fig.tight_layout()
        self.fig.show()

    def _on_joint_state(self, msg):
        positions = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                positions[name] = msg.position[i]
        if not all(name in positions for name in self.joint_names):
            return

        feedback = np.array([float(positions[name]) for name in self.joint_names])
        with self.state_lock:
            self.latest_feedback = feedback
            self.latest_state_time = self.get_clock().now()

    def _on_controller_state(self, msg):
        if not msg.joint_names or len(msg.reference.positions) < len(msg.joint_names):
            return
        references = {}
        feedback = {}
        for i, name in enumerate(msg.joint_names):
            if i < len(msg.reference.positions):
                references[name] = msg.reference.positions[i]
            if i < len(msg.feedback.positions):
                feedback[name] = msg.feedback.positions[i]

        with self.state_lock:
            if all(name in references for name in self.joint_names):
                self.latest_reference = np.array(
                    [float(references[name]) for name in self.joint_names]
                )
                self.latest_reference_time = self.get_clock().now()
            if all(name in feedback for name in self.joint_names):
                self.latest_feedback = np.array(
                    [float(feedback[name]) for name in self.joint_names]
                )
                self.latest_state_time = self.get_clock().now()

    def _sample(self):
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        with self.state_lock:
            feedback = None if self.latest_feedback is None else self.latest_feedback.copy()
            reference = None if self.latest_reference is None else self.latest_reference.copy()
            state_time = self.latest_state_time
            reference_time = self.latest_reference_time

        if feedback is None or state_time is None:
            return
        state_age = (now - state_time).nanoseconds * 1e-9
        if state_age > self.state_timeout_sec:
            self.get_logger().warn(
                f"Joint feedback is stale ({state_age:.3f}s); skipping sample.",
                throttle_duration_sec=2.0,
            )
            return

        if reference is None or reference_time is None:
            reference = feedback.copy()
        else:
            reference_age = (now - reference_time).nanoseconds * 1e-9
            if reference_age > self.state_timeout_sec:
                reference = feedback.copy()

        error = reference - feedback
        self._publish_error(error)
        self._record_sample(stamp, feedback, reference, error)

    def _publish_error(self, error):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in error]
        self.error_pub.publish(msg)

    def _record_sample(self, stamp, feedback, reference, error):
        self.times.append(stamp)
        self.feedback_history.append(feedback.copy())
        self.reference_history.append(reference.copy())
        self.error_history.append(error.copy())
        cutoff = stamp - self.plot_history_sec
        while self.times and self.times[0] < cutoff:
            self.times.popleft()
            self.feedback_history.popleft()
            self.reference_history.popleft()
            self.error_history.popleft()

        if self.csv_writer:
            for name, fb, ref, err in zip(self.joint_names, feedback, reference, error):
                self.csv_writer.writerow([f"{stamp:.6f}", name, f"{fb:.6f}", f"{ref:.6f}", f"{err:.6f}"])
            self.csv_file.flush()

    def _update_plot(self):
        if not self.plot_enabled or not self.times:
            return
        times = np.array(self.times)
        feedback = np.vstack(self.feedback_history)
        reference = np.vstack(self.reference_history)
        error = np.vstack(self.error_history)
        x = times - times[-1]
        for i in range(len(self.joint_names)):
            self.feedback_lines[i].set_data(x, feedback[:, i])
            self.reference_lines[i].set_data(x, reference[:, i])
            self.error_lines[i].set_data(x, error[:, i])
            self.axes[i].set_xlim(-self.plot_history_sec, 0.0)
            values = np.concatenate([feedback[:, i], reference[:, i]])
            lo = min(self.plot_min_rad, float(np.min(values)) - 0.05)
            hi = max(self.plot_max_rad, float(np.max(values)) + 0.05)
            if hi - lo < 1e-3:
                hi += 0.1
                lo -= 0.1
            self.axes[i].set_ylim(lo, hi)
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
