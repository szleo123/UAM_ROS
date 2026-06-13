#!/usr/bin/env python3

import csv
import math
import threading
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from trajectory_msgs.msg import JointTrajectory


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def parse_csv(value, cast=str):
    if isinstance(value, (list, tuple)):
        return [cast(item) for item in value]
    text = str(value).strip()
    if text.startswith("["):
        text = text[1:]
    if text.endswith("]"):
        text = text[:-1]
    return [cast(item.strip()) for item in text.split(",") if item.strip()]


def finite_or_nan(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


class TrajectoryCommandMonitor(Node):
    """Plot the final JointTrajectory command sent to ros2_control."""

    def __init__(self):
        super().__init__("trajectory_command_monitor")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter(
            "joint_names", "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6", dynamic_param
        )
        self.declare_parameter("plot", True, dynamic_param)
        self.declare_parameter("plot_history_sec", 15.0, dynamic_param)
        self.declare_parameter("plot_rate_hz", 10.0, dynamic_param)
        self.declare_parameter("position_abs_rad", 3.2, dynamic_param)
        self.declare_parameter("step_abs_rad", 0.01, dynamic_param)
        self.declare_parameter("velocity_abs_rad_s", 0.5, dynamic_param)
        self.declare_parameter("csv_enabled", False, dynamic_param)
        self.declare_parameter("csv_file", "/tmp/teleop_final_command.csv")
        self.declare_parameter("csv_append", False, dynamic_param)

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.joint_names = parse_csv(self.get_parameter("joint_names").value, str)
        self.plot_enabled = as_bool(self.get_parameter("plot").value)
        self.plot_history_sec = max(float(self.get_parameter("plot_history_sec").value), 1.0)
        self.plot_rate_hz = max(float(self.get_parameter("plot_rate_hz").value), 1.0)
        self.position_abs_rad = abs(float(self.get_parameter("position_abs_rad").value))
        self.step_abs_rad = abs(float(self.get_parameter("step_abs_rad").value))
        self.velocity_abs_rad_s = abs(float(self.get_parameter("velocity_abs_rad_s").value))
        self.csv_enabled = as_bool(self.get_parameter("csv_enabled").value)
        self.csv_path = Path(str(self.get_parameter("csv_file").value))
        self.csv_append = as_bool(self.get_parameter("csv_append").value)

        self.lock = threading.Lock()
        self.start_time: Optional[float] = None
        self.last_wall_time: Optional[float] = None
        self.last_positions: Optional[List[float]] = None
        self.times = deque()
        self.positions = deque()
        self.steps = deque()
        self.implied_velocities = deque()
        self.point_durations = deque()

        self.create_subscription(JointTrajectory, self.trajectory_topic, self.trajectory_callback, 50)

        if self.csv_enabled:
            self.open_csv()
        else:
            self.csv_file = None
            self.csv_writer = None

        if self.plot_enabled:
            self.setup_plot()
            self.create_timer(1.0 / self.plot_rate_hz, self.update_plot)

        self.get_logger().warn(
            f"Trajectory command monitor ready: topic={self.trajectory_topic}, "
            f"joints={self.joint_names}, plot={self.plot_enabled}, csv={self.csv_enabled}"
        )

    def open_csv(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "a" if self.csv_append else "w"
        exists = self.csv_path.exists() and self.csv_path.stat().st_size > 0
        self.csv_file = self.csv_path.open(mode, newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if not self.csv_append or not exists:
            self.csv_writer.writerow(
                [
                    "time_since_start_sec",
                    "joint_name",
                    "command_position_rad",
                    "command_step_rad",
                    "implied_command_velocity_rad_s",
                    "trajectory_point_duration_sec",
                ]
            )
            self.csv_file.flush()

    def setup_plot(self):
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().error(f"Matplotlib unavailable; trajectory command plot disabled: {exc}")
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
            ("position", "cmd position [rad]", (-self.position_abs_rad, self.position_abs_rad)),
            ("step", "cmd step [rad]", (-self.step_abs_rad, self.step_abs_rad)),
            (
                "velocity",
                "implied cmd vel [rad/s]",
                (-self.velocity_abs_rad_s, self.velocity_abs_rad_s),
            ),
        ]
        for row, name in enumerate(self.joint_names):
            for col, (kind, ylabel, ylim) in enumerate(columns):
                ax = self.axes[row][col]
                line, = ax.plot([], [], label=kind)
                self.lines[(kind, row)] = line
                ax.grid(True)
                ax.set_ylim(*ylim)
                if row == 0:
                    ax.set_title(kind)
                if col == 0:
                    ax.set_ylabel(name)
                if col == 2:
                    ax.yaxis.set_label_position("right")
                    ax.set_ylabel(ylabel)
        for ax in self.axes[-1]:
            ax.set_xlabel("time [s]")
        self.fig.canvas.manager.set_window_title("Final teleop trajectory command")
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        self.fig.show()
        self.fig.canvas.flush_events()
        self.plt.pause(0.001)

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points or not msg.points[0].positions:
            return

        point = msg.points[0]
        by_joint: Dict[str, float] = {
            name: finite_or_nan(position)
            for name, position in zip(msg.joint_names, point.positions)
        }
        if any(name not in by_joint for name in self.joint_names):
            return

        now = self.get_clock().now()
        wall_time = now.nanoseconds * 1.0e-9
        if self.start_time is None:
            self.start_time = wall_time
        t = wall_time - self.start_time

        positions = [by_joint[name] for name in self.joint_names]
        if self.last_positions is None:
            steps = [0.0 for _ in positions]
            implied_velocities = [0.0 for _ in positions]
        else:
            dt = max(wall_time - (self.last_wall_time or wall_time), 1.0e-6)
            steps = [
                position - previous
                for position, previous in zip(positions, self.last_positions)
            ]
            implied_velocities = [step / dt for step in steps]

        duration = point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9

        with self.lock:
            self.times.append(t)
            self.positions.append(positions)
            self.steps.append(steps)
            self.implied_velocities.append(implied_velocities)
            self.point_durations.append(duration)
            self.trim_history_locked()
            self.last_positions = positions
            self.last_wall_time = wall_time

        if self.csv_writer:
            for name, position, step, velocity in zip(
                self.joint_names, positions, steps, implied_velocities
            ):
                self.csv_writer.writerow(
                    [f"{t:.6f}", name, f"{position:.9f}", f"{step:.9f}", f"{velocity:.9f}", f"{duration:.6f}"]
                )
            self.csv_file.flush()

    def trim_history_locked(self):
        if not self.times:
            return
        newest = self.times[-1]
        while self.times and newest - self.times[0] > self.plot_history_sec:
            self.times.popleft()
            self.positions.popleft()
            self.steps.popleft()
            self.implied_velocities.popleft()
            self.point_durations.popleft()

    def update_plot(self):
        if not self.plot_enabled:
            return
        if not self.times:
            self.fig.canvas.flush_events()
            self.plt.pause(0.001)
            return
        with self.lock:
            x = list(self.times)
            positions = list(self.positions)
            steps = list(self.steps)
            velocities = list(self.implied_velocities)

        for row in range(len(self.joint_names)):
            self.lines[("position", row)].set_data(x, [sample[row] for sample in positions])
            self.lines[("step", row)].set_data(x, [sample[row] for sample in steps])
            self.lines[("velocity", row)].set_data(x, [sample[row] for sample in velocities])

        xmin = max(0.0, x[-1] - self.plot_history_sec)
        xmax = max(self.plot_history_sec, x[-1])
        for ax in self.axes.flat:
            ax.set_xlim(xmin, xmax)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def destroy_node(self):
        if getattr(self, "csv_file", None):
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryCommandMonitor()
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
