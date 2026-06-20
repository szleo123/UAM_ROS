#!/usr/bin/env python3

import csv
import math
import signal
from pathlib import Path
import re
from time import strftime
import tkinter as tk
from tkinter import simpledialog

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TwistStamped
from my_arm_hardware.srv import GripperStatus
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import String, UInt8
from trajectory_msgs.msg import JointTrajectory


HOMING_LABELS = {
    0: "System Starting",
    1: "Safe Heartbeat",
    2: "Waiting Drop Pose",
    3: "Zeroing Joint 3",
    4: "Arm Ready",
}

STM32_LABELS = {
    0: "WAIT_CONNECT",
    1: "RUNNING",
    2: "SAFE_DROP",
}

READY_HOMING_STATE = 4
ACTIVE_GOAL_STATES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def now_sec(node):
    return node.get_clock().now().nanoseconds * 1.0e-9


def parse_status_text(text):
    values = {}
    for part in str(text).split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values))


def msg_time_sec(node, stamp):
    if stamp.sec == 0 and stamp.nanosec == 0:
        return now_sec(node)
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def newest_stm32_trace(directory):
    path = Path(directory).expanduser()
    if not path.exists():
        return None
    candidates = [
        item
        for item in path.glob("*.csv")
        if item.is_file()
        and not item.name.endswith("_state_machine.csv")
        and not item.name.endswith("_pipeline_log.csv")
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda item: item.stat().st_mtime)


def sanitize_label(value):
    label = str(value).strip()
    if not label:
        return ""
    label = re.sub(r"[^A-Za-z0-9_.-]+", "_", label)
    return label.strip("._-")


class StateMachineNode(Node):
    def __init__(self):
        super().__init__("state_machine_dashboard")

        self.declare_parameter("state_log_directory", "/home/li/UAM_ROS/run_logs")
        self.declare_parameter("state_log_file", "")
        self.declare_parameter("state_log_run_label", "")
        self.declare_parameter("state_log_append", False)
        self.declare_parameter("state_log_match_latest_stm32", True)
        self.declare_parameter("state_log_enabled", True)
        self.declare_parameter("state_log_heartbeat_s", 10.0)
        self.declare_parameter("pipeline_log_enabled", True)
        self.declare_parameter("pipeline_log_file", "")
        self.declare_parameter("pipeline_markdown_file", "")
        self.declare_parameter("gripper_status_service", "/gripper/query_status")
        self.declare_parameter("gripper_status_poll_s", 1.0)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_closed_position", -0.69)
        self.declare_parameter("gripper_position_tolerance", 0.04)
        self.declare_parameter("topic_timeout_s", 1.0)
        self.declare_parameter("trajectory_active_timeout_s", 0.75)

        self.log_directory = str(self.get_parameter("state_log_directory").value)
        self.log_file_param = str(self.get_parameter("state_log_file").value)
        self.log_run_label = sanitize_label(
            self.get_parameter("state_log_run_label").value
        )
        self.log_append = as_bool(self.get_parameter("state_log_append").value)
        self.log_match_latest = as_bool(
            self.get_parameter("state_log_match_latest_stm32").value
        )
        self.log_enabled = as_bool(self.get_parameter("state_log_enabled").value)
        self.log_heartbeat_s = max(
            1.0, float(self.get_parameter("state_log_heartbeat_s").value)
        )
        self.pipeline_log_enabled = as_bool(
            self.get_parameter("pipeline_log_enabled").value
        )
        self.pipeline_log_file_param = str(self.get_parameter("pipeline_log_file").value)
        self.pipeline_markdown_file_param = str(
            self.get_parameter("pipeline_markdown_file").value
        )
        self.gripper_status_service = str(
            self.get_parameter("gripper_status_service").value
        )
        self.gripper_status_poll_s = max(
            0.25, float(self.get_parameter("gripper_status_poll_s").value)
        )
        self.gripper_open_position = float(
            self.get_parameter("gripper_open_position").value
        )
        self.gripper_closed_position = float(
            self.get_parameter("gripper_closed_position").value
        )
        self.gripper_position_tolerance = abs(
            float(self.get_parameter("gripper_position_tolerance").value)
        )
        self.topic_timeout_s = max(0.1, float(self.get_parameter("topic_timeout_s").value))
        self.trajectory_active_timeout_s = max(
            0.1, float(self.get_parameter("trajectory_active_timeout_s").value)
        )

        self.homing_state = None
        self.homing_text = "Waiting for homing status"
        self.stm32_state = None
        self.safety_status_text = ""
        self.safety_status = {}
        self.gate_status_text = ""
        self.gate_status = {}
        self.last_raw_twist_time = None
        self.last_filtered_twist_time = None
        self.last_servo_trajectory_time = None
        self.last_final_trajectory_time = None
        self.last_final_trajectory_source = "unknown"
        self.controller_action_active = False
        self.controller_status_text = "No controller action status"
        self.gripper_backend = "unknown"
        self.gripper_state = "Waiting"
        self.gripper_notes = "Waiting for gripper service"
        self.gripper_connected = False
        self.last_gripper_poll = 0.0
        self.gripper_future = None
        self.last_snapshot = None
        self.last_log_time = 0.0
        self.csv_file = None
        self.csv_writer = None
        self.log_path = None
        self.pipeline_csv_file = None
        self.pipeline_csv_writer = None
        self.pipeline_csv_path = None
        self.pipeline_md_file = None
        self.pipeline_md_path = None
        self.pipeline_row_count = 0
        self.latest_pipeline_event = "No pipeline evidence recorded yet."

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(UInt8, "/arm_homing/state", self.on_homing_state, latched_qos)
        self.create_subscription(
            String, "/arm_homing/status_text", self.on_homing_text, latched_qos
        )
        self.create_subscription(
            UInt8, "/arm_homing/stm32_sys_state", self.on_stm32_state, latched_qos
        )
        self.create_subscription(String, "/arm_teleop/safety_status", self.on_safety_status, 10)
        self.create_subscription(
            String, "/arm_teleop/trajectory_gate_status", self.on_gate_status, 10
        )
        self.create_subscription(TwistStamped, "/arm_teleop/raw_twist_cmd", self.on_raw_twist, 10)
        self.create_subscription(TwistStamped, "/arm_teleop/twist_cmd", self.on_filtered_twist, 10)
        self.create_subscription(
            JointTrajectory,
            "/arm_teleop/servo_raw_joint_trajectory",
            self.on_servo_trajectory,
            10,
        )
        self.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory", self.on_final_trajectory, 50
        )
        self.create_subscription(
            GoalStatusArray,
            "/arm_controller/follow_joint_trajectory/_action/status",
            self.on_controller_status,
            10,
        )

        self.gripper_client = self.create_client(GripperStatus, self.gripper_status_service)
        self.create_timer(0.25, self.timer_callback)

        self.open_log()
        if self.log_path:
            self.get_logger().warn(f"State machine log: {self.log_path}")
        self.open_pipeline_log()
        if self.pipeline_csv_path:
            self.get_logger().warn(f"Pipeline evidence log: {self.pipeline_csv_path}")
            self.record_pipeline_event(
                "00",
                "Dashboard Started",
                "auto",
                "pass",
                "Read-only evidence logger started.",
            )
        self.get_logger().warn("State machine dashboard observer ready.")

    def open_log(self):
        if not self.log_enabled:
            return

        if self.log_file_param:
            path = Path(self.log_file_param).expanduser()
        else:
            trace = newest_stm32_trace(self.log_directory) if self.log_match_latest else None
            if trace is not None:
                stem = trace.stem
                if self.log_run_label and self.log_run_label not in stem:
                    stem = f"{stem}_{self.log_run_label}"
                path = trace.with_name(f"{stem}_state_machine.csv")
            else:
                stem = strftime("%Y%m%d_%H%M%S")
                if self.log_run_label:
                    stem = f"{stem}_{self.log_run_label}"
                path = Path(self.log_directory).expanduser() / f"{stem}_state_machine.csv"

        path.parent.mkdir(parents=True, exist_ok=True)
        mode = "a" if self.log_append else "w"
        existed = path.exists() and path.stat().st_size > 0
        self.csv_file = path.open(mode, newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.log_path = path
        if not self.log_append or not existed:
            self.csv_writer.writerow(
                [
                    "time_sec",
                    "arm_state",
                    "stm32_state",
                    "command_source",
                    "planned_state",
                    "teleop_state",
                    "gripper_backend",
                    "gripper_state",
                    "notes",
                ]
            )
            self.csv_file.flush()

    def default_pipeline_base_path(self):
        if self.log_path is not None:
            stem = self.log_path.stem
            if stem.endswith("_state_machine"):
                stem = stem[: -len("_state_machine")]
            return self.log_path.with_name(f"{stem}_pipeline_log")

        stem = strftime("%Y%m%d_%H%M%S")
        if self.log_run_label:
            stem = f"{stem}_{self.log_run_label}"
        return Path(self.log_directory).expanduser() / f"{stem}_pipeline_log"

    def open_pipeline_log(self):
        if not self.pipeline_log_enabled:
            return

        base_path = self.default_pipeline_base_path()
        csv_path = (
            Path(self.pipeline_log_file_param).expanduser()
            if self.pipeline_log_file_param
            else base_path.with_suffix(".csv")
        )
        md_path = (
            Path(self.pipeline_markdown_file_param).expanduser()
            if self.pipeline_markdown_file_param
            else base_path.with_suffix(".md")
        )

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        md_path.parent.mkdir(parents=True, exist_ok=True)
        csv_exists = csv_path.exists() and csv_path.stat().st_size > 0
        md_exists = md_path.exists() and md_path.stat().st_size > 0

        mode = "a" if self.log_append else "w"
        self.pipeline_csv_file = csv_path.open(mode, newline="", encoding="utf-8")
        self.pipeline_csv_writer = csv.writer(self.pipeline_csv_file)
        self.pipeline_csv_path = csv_path
        if not self.log_append or not csv_exists:
            self.pipeline_csv_writer.writerow(
                [
                    "time_sec",
                    "step",
                    "event",
                    "source",
                    "result",
                    "operator_note",
                    "arm_state",
                    "stm32_state",
                    "command_source",
                    "planned_state",
                    "teleop_state",
                    "gripper_backend",
                    "gripper_state",
                    "system_notes",
                ]
            )
            self.pipeline_csv_file.flush()

        self.pipeline_md_file = md_path.open(mode, encoding="utf-8")
        self.pipeline_md_path = md_path
        if not self.log_append or not md_exists:
            self.pipeline_md_file.write("# Human-In-The-Loop Sampling Pipeline Log\n\n")
            if self.log_run_label:
                self.pipeline_md_file.write(f"Run label: `{self.log_run_label}`\n\n")
            self.pipeline_md_file.write(
                "| Time [s] | Step | Event | Source | Result | Operator Note | "
                "Arm | Teleop | Gripper |\n"
            )
            self.pipeline_md_file.write(
                "|---:|---|---|---|---|---|---|---|---|\n"
            )
            self.pipeline_md_file.flush()

    def record_pipeline_event(self, step, event, source="manual", result="marked", note=""):
        snapshot = self.snapshot()
        self.latest_pipeline_event = f"{step} {event}: {result}"
        if note:
            self.latest_pipeline_event += f" - {note}"

        row = [
            snapshot["time_sec"],
            step,
            event,
            source,
            result,
            note,
            snapshot["arm_state"],
            snapshot["stm32_state"],
            snapshot["command_source"],
            snapshot["planned_state"],
            snapshot["teleop_state"],
            snapshot["gripper_backend"],
            snapshot["gripper_state"],
            snapshot["notes"],
        ]

        if self.pipeline_csv_writer is not None:
            self.pipeline_csv_writer.writerow(row)
            self.pipeline_csv_file.flush()

        if self.pipeline_md_file is not None:
            self.pipeline_md_file.write(
                "| "
                + " | ".join(
                    [
                        snapshot["time_sec"],
                        step,
                        event,
                        source,
                        result,
                        note.replace("|", "/"),
                        snapshot["arm_state"],
                        snapshot["teleop_state"],
                        f"{snapshot['gripper_backend']} {snapshot['gripper_state']}",
                    ]
                )
                + " |\n"
            )
            self.pipeline_md_file.flush()

        self.pipeline_row_count += 1
        self.get_logger().warn(f"Pipeline evidence marked: {self.latest_pipeline_event}")

    def on_homing_state(self, msg):
        self.homing_state = int(msg.data)

    def on_homing_text(self, msg):
        self.homing_text = msg.data

    def on_stm32_state(self, msg):
        self.stm32_state = int(msg.data)

    def on_safety_status(self, msg):
        self.safety_status_text = msg.data
        self.safety_status = parse_status_text(msg.data)

    def on_gate_status(self, msg):
        self.gate_status_text = msg.data
        self.gate_status = parse_status_text(msg.data)

    def on_raw_twist(self, msg):
        self.last_raw_twist_time = msg_time_sec(self, msg.header.stamp)

    def on_filtered_twist(self, msg):
        self.last_filtered_twist_time = msg_time_sec(self, msg.header.stamp)

    def on_servo_trajectory(self, _msg):
        self.last_servo_trajectory_time = now_sec(self)

    def on_final_trajectory(self, _msg):
        now = now_sec(self)
        self.last_final_trajectory_time = now
        if self.last_servo_trajectory_time is not None and (
            now - self.last_servo_trajectory_time
        ) <= self.trajectory_active_timeout_s:
            self.last_final_trajectory_source = "teleop"
        else:
            self.last_final_trajectory_source = "planned"

    def on_controller_status(self, msg):
        active = False
        labels = []
        for status in msg.status_list:
            if status.status in ACTIVE_GOAL_STATES:
                active = True
                labels.append(str(status.status))
        self.controller_action_active = active
        self.controller_status_text = "active" if active else "idle"
        if labels:
            self.controller_status_text += f" ({','.join(labels)})"

    def timer_callback(self):
        self.poll_gripper()
        self.log_if_needed()

    def poll_gripper(self):
        t = now_sec(self)
        if self.gripper_future is not None:
            if self.gripper_future.done():
                future = self.gripper_future
                self.gripper_future = None
                self.handle_gripper_response(future)
            return

        if t - self.last_gripper_poll < self.gripper_status_poll_s:
            return
        self.last_gripper_poll = t

        if not self.gripper_client.service_is_ready():
            self.gripper_connected = False
            self.gripper_backend = "unknown"
            self.gripper_state = "Disconnected"
            self.gripper_notes = f"Service unavailable: {self.gripper_status_service}"
            return

        self.gripper_future = self.gripper_client.call_async(GripperStatus.Request())

    def handle_gripper_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.gripper_connected = False
            self.gripper_state = "Status Error"
            self.gripper_notes = str(exc)
            return

        self.gripper_backend = response.backend or "unknown"
        self.gripper_connected = bool(response.connected and response.success)
        if not response.success:
            self.gripper_state = "Status Error"
            self.gripper_notes = response.message
            return
        if not response.connected:
            self.gripper_state = "Disconnected"
            self.gripper_notes = response.message
            return

        faults = []
        if response.stall_protection:
            faults.append("stall")
        if response.over_current:
            faults.append("over_current")
        if response.over_temperature:
            faults.append("over_temperature")
        if response.motor_abnormal:
            faults.append("motor_abnormal")
        if faults:
            self.gripper_state = "Fault / Protection"
            self.gripper_notes = ",".join(faults)
            return

        position = float(response.position)
        if abs(position - self.gripper_open_position) <= self.gripper_position_tolerance:
            self.gripper_state = "Open"
        elif abs(position - self.gripper_closed_position) <= self.gripper_position_tolerance:
            self.gripper_state = "Closed / Sampling"
        else:
            self.gripper_state = "Holding"
        self.gripper_notes = (
            f"pos={position:.3f}, current={response.current_ma}mA, "
            f"temp={response.temperature_c:.0f}C"
        )

    def arm_state_label(self):
        if self.homing_state is None:
            return "Unknown"
        return HOMING_LABELS.get(self.homing_state, f"State {self.homing_state}")

    def stm32_state_label(self):
        if self.stm32_state is None:
            return "Unknown"
        return STM32_LABELS.get(self.stm32_state, f"SYS {self.stm32_state}")

    def teleop_state_label(self):
        t = now_sec(self)
        reason = self.gate_status.get("reason") or self.safety_status.get("reason")
        deadman = self.gate_status.get("deadman") or self.safety_status.get("deadman")
        command = self.gate_status.get("command")

        if not self.safety_status and not self.gate_status:
            return "Offline"
        if reason == "allowed":
            return "Commanding"
        if deadman == "0":
            return "Waiting Deadman"
        if command == "0":
            return "Waiting Command"
        if reason:
            return f"Blocked: {reason}"
        if self.last_filtered_twist_time is not None and t - self.last_filtered_twist_time < self.topic_timeout_s:
            return "Active"
        return "Idle"

    def planned_state_label(self):
        t = now_sec(self)
        if self.controller_action_active:
            return "Executing"
        if (
            self.last_final_trajectory_time is not None
            and t - self.last_final_trajectory_time < self.trajectory_active_timeout_s
        ):
            return "Trajectory Sent"
        return "Idle"

    def command_source_label(self):
        t = now_sec(self)
        teleop_recent = (
            self.last_final_trajectory_source == "teleop"
            and self.last_final_trajectory_time is not None
            and t - self.last_final_trajectory_time < self.trajectory_active_timeout_s
        )
        planned_recent = (
            self.last_final_trajectory_source == "planned"
            and self.last_final_trajectory_time is not None
            and t - self.last_final_trajectory_time < self.trajectory_active_timeout_s
        )
        if teleop_recent:
            return "Teleop"
        if planned_recent or self.controller_action_active:
            return "Planned / Controller"
        return "None"

    def notes_text(self):
        parts = []
        if self.homing_text:
            parts.append(self.homing_text)
        if self.gate_status_text:
            parts.append(f"gate: {self.gate_status_text}")
        if self.gripper_notes:
            parts.append(f"gripper: {self.gripper_notes}")
        return " | ".join(parts)

    def snapshot(self):
        return {
            "time_sec": f"{now_sec(self):.6f}",
            "arm_state": self.arm_state_label(),
            "stm32_state": self.stm32_state_label(),
            "command_source": self.command_source_label(),
            "planned_state": self.planned_state_label(),
            "teleop_state": self.teleop_state_label(),
            "gripper_backend": self.gripper_backend,
            "gripper_state": self.gripper_state,
            "notes": self.notes_text(),
        }

    def log_if_needed(self):
        if self.csv_writer is None:
            return
        snapshot = self.snapshot()
        key = tuple(
            snapshot[name]
            for name in (
                "arm_state",
                "stm32_state",
                "command_source",
                "planned_state",
                "teleop_state",
                "gripper_backend",
                "gripper_state",
            )
        )
        t = float(snapshot["time_sec"])
        if key == self.last_snapshot and t - self.last_log_time < self.log_heartbeat_s:
            return
        self.last_snapshot = key
        self.last_log_time = t
        self.csv_writer.writerow(
            [
                snapshot["time_sec"],
                snapshot["arm_state"],
                snapshot["stm32_state"],
                snapshot["command_source"],
                snapshot["planned_state"],
                snapshot["teleop_state"],
                snapshot["gripper_backend"],
                snapshot["gripper_state"],
                snapshot["notes"],
            ]
        )
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        if self.pipeline_csv_file is not None:
            self.pipeline_csv_file.close()
        if self.pipeline_md_file is not None:
            self.pipeline_md_file.close()
        super().destroy_node()


class StateMachineDashboard:
    def __init__(self, node):
        self.node = node
        self.closed = False
        self.after_id = None

        self.root = tk.Tk()
        self.root.title("UAM Sampling State Machine")
        self.root.geometry("1180x680")
        self.root.minsize(1040, 620)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self.request_shutdown)
        signal.signal(signal.SIGTERM, self.request_shutdown)

        self.status_var = tk.StringVar(value="Starting dashboard...")
        self.log_var = tk.StringVar(value="Log: disabled")
        self.pipeline_var = tk.StringVar(value=self.node.latest_pipeline_event)

        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(main_frame, bg="#f7f8fa", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.pipeline_frame = tk.Frame(main_frame, width=300, padx=12, pady=14, bg="#ffffff")
        self.pipeline_frame.pack(side=tk.RIGHT, fill=tk.Y)
        self.pipeline_frame.pack_propagate(False)
        self.build_pipeline_panel()

        footer = tk.Frame(self.root, padx=12, pady=8)
        footer.pack(fill=tk.X)
        tk.Label(footer, textvariable=self.status_var, anchor="w", justify=tk.LEFT).pack(
            side=tk.LEFT, fill=tk.X, expand=True
        )
        tk.Label(footer, textvariable=self.log_var, anchor="e", justify=tk.RIGHT).pack(
            side=tk.RIGHT
        )
        self.root.bind("<Configure>", lambda _event: self.draw())

        self.colors = {
            "offline": "#d5d9df",
            "waiting": "#ffe08a",
            "ready": "#9ee6a8",
            "active": "#8ecbff",
            "fault": "#ff9f9f",
            "teleop": "#c3b5ff",
        }
        self.draw()
        self.schedule_update()

    def build_pipeline_panel(self):
        tk.Label(
            self.pipeline_frame,
            text="Pipeline Evidence Log",
            bg="#ffffff",
            fg="#111827",
            font=("Sans", 14, "bold"),
            anchor="w",
        ).pack(fill=tk.X)
        tk.Label(
            self.pipeline_frame,
            text="Log-only buttons. They do not command the robot.",
            bg="#ffffff",
            fg="#4b5563",
            wraplength=260,
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, pady=(4, 12))

        buttons = [
            ("01", "Target Identified", "pass"),
            ("02", "Pre-Sampling Pose Reached", "pass"),
            ("03", "Human Takeover", "pass"),
            ("04", "Sampling Attempt", "started"),
            ("05", "Sampling Success", "success"),
            ("06", "Sampling Failed", "failed"),
            ("07", "Retry Started", "started"),
            ("08", "Return / Retract Started", "started"),
            ("09", "Return Complete", "pass"),
        ]
        for step, label, result in buttons:
            tk.Button(
                self.pipeline_frame,
                text=label,
                anchor="w",
                command=lambda s=step, e=label, r=result: self.mark_pipeline(s, e, r),
            ).pack(fill=tk.X, pady=2)

        tk.Button(
            self.pipeline_frame,
            text="Add Note",
            anchor="w",
            command=self.add_pipeline_note,
        ).pack(fill=tk.X, pady=(12, 2))

        tk.Label(
            self.pipeline_frame,
            text="Latest evidence row",
            bg="#ffffff",
            fg="#111827",
            font=("Sans", 10, "bold"),
            anchor="w",
        ).pack(fill=tk.X, pady=(18, 4))
        tk.Label(
            self.pipeline_frame,
            textvariable=self.pipeline_var,
            bg="#ffffff",
            fg="#374151",
            wraplength=260,
            justify=tk.LEFT,
            anchor="nw",
        ).pack(fill=tk.X)

    def mark_pipeline(self, step, event, result):
        self.node.record_pipeline_event(step, event, "manual", result, "")
        self.pipeline_var.set(self.node.latest_pipeline_event)

    def add_pipeline_note(self):
        note = simpledialog.askstring(
            "Add Pipeline Note",
            "Note to add to the pipeline evidence log:",
            parent=self.root,
        )
        if note is None:
            return
        note = note.strip()
        if not note:
            return
        self.node.record_pipeline_event("NOTE", "Operator Note", "manual", "note", note)
        self.pipeline_var.set(self.node.latest_pipeline_event)

    def request_shutdown(self, *_args):
        self.close()

    def close(self):
        self.closed = True
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except tk.TclError:
                pass
        self.root.quit()

    def schedule_update(self):
        if self.closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except (ExternalShutdownException, RCLError):
            self.close()
            return
        self.draw()
        self.after_id = self.root.after(100, self.schedule_update)

    def node_color(self, kind, text):
        lower = text.lower()
        if any(word in lower for word in ("fault", "error", "safe_drop", "blocked")):
            return self.colors["fault"]
        if any(word in lower for word in ("ready", "open", "idle")) and kind != "teleop":
            return self.colors["ready"]
        if any(word in lower for word in ("waiting", "starting", "unknown", "disconnected")):
            return self.colors["waiting"]
        if kind == "teleop" and any(word in lower for word in ("commanding", "active")):
            return self.colors["teleop"]
        if any(word in lower for word in ("executing", "sent", "commanding", "active")):
            return self.colors["active"]
        return "#ffffff"

    def draw_node(self, x, y, w, h, title, state, detail="", kind="normal"):
        color = self.node_color(kind, state)
        self.canvas.create_rectangle(
            x, y, x + w, y + h, fill=color, outline="#27313f", width=2
        )
        self.canvas.create_text(
            x + 14, y + 14, text=title, anchor="nw", fill="#27313f", font=("Sans", 13, "bold")
        )
        self.canvas.create_text(
            x + 14, y + 42, text=state, anchor="nw", fill="#111827", font=("Sans", 16, "bold")
        )
        if detail:
            self.canvas.create_text(
                x + 14,
                y + 76,
                text=detail,
                anchor="nw",
                fill="#364152",
                width=max(40, w - 28),
                font=("Sans", 10),
            )

    def arrow(self, x1, y1, x2, y2, text=""):
        self.canvas.create_line(x1, y1, x2, y2, arrow=tk.LAST, width=2, fill="#566273")
        if text:
            self.canvas.create_text(
                (x1 + x2) / 2.0,
                (y1 + y2) / 2.0 - 12,
                text=text,
                fill="#566273",
                font=("Sans", 9, "bold"),
            )

    def both_arrow(self, x1, y1, x2, y2, text=""):
        self.canvas.create_line(x1, y1, x2, y2, arrow=tk.BOTH, width=2, fill="#566273")
        if text:
            self.canvas.create_text(
                (x1 + x2) / 2.0,
                (y1 + y2) / 2.0 - 12,
                text=text,
                fill="#566273",
                font=("Sans", 9, "bold"),
            )

    def draw(self):
        if self.closed:
            return
        width = max(self.canvas.winfo_width(), 860)
        self.canvas.delete("all")
        self.canvas.create_text(
            24,
            20,
            text="Human-in-the-loop automatic sampling state machine",
            anchor="nw",
            fill="#111827",
            font=("Sans", 18, "bold"),
        )

        snapshot = self.node.snapshot()
        init_state = snapshot["arm_state"]
        stm32_state = snapshot["stm32_state"]
        planned_state = snapshot["planned_state"]
        teleop_state = snapshot["teleop_state"]
        command_source = snapshot["command_source"]
        gripper_state = snapshot["gripper_state"]

        left = 40
        top = 90
        box_w = min(230, max(190, (width - 160) // 3))
        box_h = 132
        gap = max(28, (width - 2 * left - 3 * box_w) // 2)
        x1 = left
        x2 = x1 + box_w + gap
        x3 = x2 + box_w + gap

        self.draw_node(
            x1,
            top,
            box_w,
            box_h,
            "Initialization / Homing",
            init_state,
            f"STM32: {stm32_state}",
        )
        self.draw_node(
            x2,
            top,
            box_w,
            box_h,
            "Arm Ready",
            "Ready" if self.node.homing_state == READY_HOMING_STATE else "Locked",
            "Motion commands are released after homing.",
        )
        self.draw_node(
            x3,
            top,
            box_w,
            box_h,
            "Planned Motion",
            planned_state,
            f"Source: {command_source}",
        )
        self.arrow(x1 + box_w, top + box_h / 2, x2, top + box_h / 2, "zero joint 3")
        self.arrow(x2 + box_w, top + box_h / 2, x3, top + box_h / 2, "execute")

        teleop_y = top + 190
        self.draw_node(
            x3,
            teleop_y,
            box_w,
            box_h,
            "Teleop Control",
            teleop_state,
            self.node.gate_status.get("reason", self.node.safety_status.get("reason", "")),
            kind="teleop",
        )
        self.both_arrow(
            x3 + box_w / 2,
            top + box_h,
            x3 + box_w / 2,
            teleop_y,
            "human override",
        )

        gripper_y = top + 190
        self.draw_node(
            x1,
            gripper_y,
            box_w,
            box_h,
            "Gripper State",
            gripper_state,
            f"{self.node.gripper_backend}: {self.node.gripper_notes}",
        )

        estop_x = x2
        estop_y = teleop_y
        self.draw_node(
            estop_x,
            estop_y,
            box_w,
            box_h,
            "Safety / Emergency Stop",
            "Available",
            "Triggered by operator UI or teleop gate block.",
        )
        self.arrow(estop_x + box_w / 2, estop_y, x2 + box_w / 2, top + box_h, "hold")

        legend_y = teleop_y + box_h + 42
        legend = [
            ("Ready", self.colors["ready"]),
            ("Waiting", self.colors["waiting"]),
            ("Active", self.colors["active"]),
            ("Teleop", self.colors["teleop"]),
            ("Fault / Blocked", self.colors["fault"]),
        ]
        x = 42
        for label, color in legend:
            self.canvas.create_rectangle(x, legend_y, x + 18, legend_y + 18, fill=color, outline="#27313f")
            self.canvas.create_text(x + 26, legend_y + 9, text=label, anchor="w", fill="#27313f")
            x += 150

        self.status_var.set(
            f"Arm: {init_state} | Source: {command_source} | "
            f"Teleop: {teleop_state} | Gripper: {self.node.gripper_backend} {gripper_state}"
        )
        self.pipeline_var.set(self.node.latest_pipeline_event)
        log_parts = []
        if self.node.log_path:
            log_parts.append(f"State: {self.node.log_path.name}")
        if self.node.pipeline_csv_path:
            log_parts.append(f"Pipeline: {self.node.pipeline_csv_path.name}")
        self.log_var.set(" | ".join(log_parts) if log_parts else "Log: disabled")

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = StateMachineNode()
    app = None
    try:
        app = StateMachineDashboard(node)
        app.run()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if app is not None:
            app.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
