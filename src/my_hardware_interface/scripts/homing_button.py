#!/usr/bin/env python3

import signal
import tkinter as tk
from tkinter import messagebox

from action_msgs.srv import CancelGoal
from control_msgs.action import GripperCommand
from my_arm_hardware.srv import GetGripperProtection, GripperStatus, SetGripperProtection
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


WAITING_INIT_COMMAND = 0
WAITING_OPERATOR_DROP_POSE = 2
WAITING_ALL_READY = 3
ALL_READY = 4

STM32_STATE_LABELS = {
    0: "WAIT_CONNECT",
    1: "RUNNING",
    2: "SAFE_DROP",
}

STM32_STATE_COLORS = {
    0: "#d7d7d7",
    1: "#b7e4b7",
    2: "#f3b3b3",
}


def as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class HomingButtonNode(Node):
    def __init__(self):
        super().__init__("arm_homing_button")
        self.state = None
        self.stm32_state = None
        self.status = "Waiting for homing status..."
        self.declare_parameter("initial_pose_trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter(
            "initial_pose_joints",
            "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6",
        )
        self.declare_parameter("initial_pose_positions", "0.0,0.0,0.0,0.0,0.0,0.0")
        self.declare_parameter("initial_pose_duration_s", 8.0)
        self.declare_parameter("enable_emergency_arm_stop", True)
        self.declare_parameter("arm_emergency_stop_service", "/arm_emergency_stop/trigger")
        self.declare_parameter("enable_emergency_gripper_open", True)
        self.declare_parameter("gripper_action_name", "/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_max_effort", 0.0)
        self.declare_parameter("gripper_cancel_before_open", True)
        self.declare_parameter("enable_gripper_maintenance_ui", True)
        self.declare_parameter("enable_gripper_parameter_writes", False)
        self.declare_parameter("enable_gripper_flash_save", False)
        self.declare_parameter("gripper_status_service", "/gripper/query_status")
        self.declare_parameter("gripper_get_protection_service", "/gripper/get_protection")
        self.declare_parameter("gripper_set_protection_service", "/gripper/set_protection")
        self.declare_parameter("gripper_clear_fault_service", "/gripper/clear_fault")
        self.declare_parameter(
            "gripper_clear_fault_open_service", "/gripper/clear_fault_and_open"
        )
        self.declare_parameter("gripper_save_parameters_service", "/gripper/save_parameters")
        self.declare_parameter("gripper_status_poll_s", 1.0)
        self.initial_pose_topic = self.get_parameter("initial_pose_trajectory_topic").value
        self.initial_pose_joints = self._parse_string_list(
            self.get_parameter("initial_pose_joints").value
        )
        self.initial_pose_positions = self._parse_float_list(
            self.get_parameter("initial_pose_positions").value
        )
        self.initial_pose_duration_s = float(self.get_parameter("initial_pose_duration_s").value)
        self.enable_emergency_arm_stop = as_bool(
            self.get_parameter("enable_emergency_arm_stop").value
        )
        self.arm_emergency_stop_service = self.get_parameter("arm_emergency_stop_service").value
        self.enable_emergency_gripper_open = as_bool(
            self.get_parameter("enable_emergency_gripper_open").value
        )
        self.gripper_action_name = self.get_parameter("gripper_action_name").value
        self.gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self.gripper_max_effort = float(self.get_parameter("gripper_max_effort").value)
        self.gripper_cancel_before_open = as_bool(
            self.get_parameter("gripper_cancel_before_open").value
        )
        self.enable_gripper_maintenance_ui = as_bool(
            self.get_parameter("enable_gripper_maintenance_ui").value
        )
        self.enable_gripper_parameter_writes = as_bool(
            self.get_parameter("enable_gripper_parameter_writes").value
        )
        self.enable_gripper_flash_save = as_bool(
            self.get_parameter("enable_gripper_flash_save").value
        )
        self.gripper_status_service = self.get_parameter("gripper_status_service").value
        self.gripper_get_protection_service = self.get_parameter(
            "gripper_get_protection_service"
        ).value
        self.gripper_set_protection_service = self.get_parameter(
            "gripper_set_protection_service"
        ).value
        self.gripper_clear_fault_service = self.get_parameter(
            "gripper_clear_fault_service"
        ).value
        self.gripper_clear_fault_open_service = self.get_parameter(
            "gripper_clear_fault_open_service"
        ).value
        self.gripper_save_parameters_service = self.get_parameter(
            "gripper_save_parameters_service"
        ).value
        self.gripper_status_poll_s = max(
            0.2, float(self.get_parameter("gripper_status_poll_s").value)
        )
        self.gripper_status_text = "Gripper: waiting for status"
        self.last_gripper_status = None
        if len(self.initial_pose_positions) != len(self.initial_pose_joints):
            self.get_logger().error(
                "initial_pose_positions has %d values but initial_pose_joints has %d; "
                "Move Initial Pose will stay disabled.",
                len(self.initial_pose_positions),
                len(self.initial_pose_joints),
            )
            self.initial_pose_positions = []

        self.drop_client = self.create_client(Trigger, "/arm_homing/confirm_drop_pose")
        self.arm_stop_client = None
        if self.enable_emergency_arm_stop:
            self.arm_stop_client = self.create_client(
                Trigger, self.arm_emergency_stop_service
            )
        self.initial_pose_pub = self.create_publisher(
            JointTrajectory, self.initial_pose_topic, 10
        )
        self.gripper_action_client = None
        self.gripper_cancel_client = None
        if self.enable_emergency_gripper_open:
            self.gripper_action_client = ActionClient(
                self, GripperCommand, self.gripper_action_name
            )
            self.gripper_cancel_client = self.create_client(
                CancelGoal, self._action_cancel_service_name(self.gripper_action_name)
            )
        self.gripper_status_client = None
        self.gripper_get_protection_client = None
        self.gripper_set_protection_client = None
        self.gripper_clear_fault_client = None
        self.gripper_clear_fault_open_client = None
        self.gripper_save_parameters_client = None
        if self.enable_gripper_maintenance_ui:
            self.gripper_status_client = self.create_client(
                GripperStatus, self.gripper_status_service
            )
            self.gripper_get_protection_client = self.create_client(
                GetGripperProtection, self.gripper_get_protection_service
            )
            self.gripper_set_protection_client = self.create_client(
                SetGripperProtection, self.gripper_set_protection_service
            )
            self.gripper_clear_fault_client = self.create_client(
                Trigger, self.gripper_clear_fault_service
            )
            self.gripper_clear_fault_open_client = self.create_client(
                Trigger, self.gripper_clear_fault_open_service
            )
            self.gripper_save_parameters_client = self.create_client(
                Trigger, self.gripper_save_parameters_service
            )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/arm_homing/status_text", self._on_status, latched_qos
        )
        self.create_subscription(UInt8, "/arm_homing/state", self._on_state, latched_qos)
        self.create_subscription(
            UInt8, "/arm_homing/stm32_sys_state", self._on_stm32_state, latched_qos
        )

    @staticmethod
    def _action_cancel_service_name(action_name):
        return f"{action_name.rstrip('/')}/_action/cancel_goal"

    def _on_status(self, msg):
        self.status = msg.data

    def _on_state(self, msg):
        self.state = msg.data

    def _on_stm32_state(self, msg):
        self.stm32_state = msg.data

    def ready_for_drop_confirmation(self):
        return self.state == WAITING_OPERATOR_DROP_POSE and self.drop_client.service_is_ready()

    def ready_for_initial_pose(self):
        return (
            self.state in (WAITING_OPERATOR_DROP_POSE, ALL_READY)
            and len(self.initial_pose_positions) == len(self.initial_pose_joints)
        )

    def ready_for_emergency_gripper_open(self):
        return (
            self.enable_emergency_gripper_open
            and self.gripper_action_client is not None
            and self.gripper_action_client.server_is_ready()
        )

    def ready_for_emergency_arm_stop(self):
        return (
            self.enable_emergency_arm_stop
            and self.arm_stop_client is not None
            and self.arm_stop_client.service_is_ready()
        )

    def ready_for_gripper_status(self):
        return (
            self.enable_gripper_maintenance_ui
            and self.gripper_status_client is not None
            and self.gripper_status_client.service_is_ready()
        )

    def ready_for_gripper_clear_fault(self):
        return (
            self.enable_gripper_maintenance_ui
            and self.gripper_clear_fault_client is not None
            and self.gripper_clear_fault_client.service_is_ready()
        )

    def ready_for_gripper_clear_fault_open(self):
        return (
            self.enable_gripper_maintenance_ui
            and self.gripper_clear_fault_open_client is not None
            and self.gripper_clear_fault_open_client.service_is_ready()
        )

    def ready_for_gripper_get_protection(self):
        return (
            self.enable_gripper_maintenance_ui
            and self.gripper_get_protection_client is not None
            and self.gripper_get_protection_client.service_is_ready()
        )

    def ready_for_gripper_set_protection(self):
        return (
            self.enable_gripper_maintenance_ui
            and self.enable_gripper_parameter_writes
            and self.gripper_set_protection_client is not None
            and self.gripper_set_protection_client.service_is_ready()
        )

    def ready_for_gripper_save_parameters(self):
        return (
            self.enable_gripper_maintenance_ui
            and self.enable_gripper_flash_save
            and self.gripper_save_parameters_client is not None
            and self.gripper_save_parameters_client.service_is_ready()
        )

    def confirm_drop_pose(self, done_cb):
        request = Trigger.Request()
        future = self.drop_client.call_async(request)
        future.add_done_callback(done_cb)
        return future

    def request_emergency_arm_stop(self, done_cb):
        if not self.ready_for_emergency_arm_stop():
            self.status = (
                f"Arm emergency stop service {self.arm_emergency_stop_service} "
                "is not available."
            )
            self.get_logger().warn(self.status)
            return None

        request = Trigger.Request()
        future = self.arm_stop_client.call_async(request)
        future.add_done_callback(done_cb)
        self.status = "Arm emergency stop requested."
        self.get_logger().warn(self.status)
        return future

    def request_gripper_status(self, done_cb):
        if not self.ready_for_gripper_status():
            self.gripper_status_text = (
                f"Gripper: service unavailable ({self.gripper_status_service})"
            )
            return None
        future = self.gripper_status_client.call_async(GripperStatus.Request())
        future.add_done_callback(done_cb)
        return future

    def request_gripper_get_protection(self, done_cb):
        if not self.ready_for_gripper_get_protection():
            self.status = (
                f"Gripper protection service {self.gripper_get_protection_service} "
                "is not available."
            )
            return None
        future = self.gripper_get_protection_client.call_async(
            GetGripperProtection.Request()
        )
        future.add_done_callback(done_cb)
        return future

    def request_gripper_clear_fault(self, done_cb):
        if not self.ready_for_gripper_clear_fault():
            self.status = (
                f"Gripper clear-fault service {self.gripper_clear_fault_service} "
                "is not available."
            )
            return None
        future = self.gripper_clear_fault_client.call_async(Trigger.Request())
        future.add_done_callback(done_cb)
        self.status = "Gripper clear-fault requested."
        return future

    def request_gripper_clear_fault_open(self, done_cb):
        if not self.ready_for_gripper_clear_fault_open():
            self.status = (
                f"Gripper clear-fault-open service "
                f"{self.gripper_clear_fault_open_service} is not available."
            )
            return None

        if (
            self.gripper_cancel_before_open
            and self.gripper_cancel_client is not None
            and self.gripper_cancel_client.service_is_ready()
        ):
            cancel_future = self.gripper_cancel_client.call_async(CancelGoal.Request())
            cancel_future.add_done_callback(
                lambda _future: self._call_gripper_clear_fault_open(done_cb)
            )
            self.status = "Canceling gripper goals before clear-fault-open."
            return cancel_future

        return self._call_gripper_clear_fault_open(done_cb)

    def _call_gripper_clear_fault_open(self, done_cb):
        future = self.gripper_clear_fault_open_client.call_async(Trigger.Request())
        future.add_done_callback(done_cb)
        self.status = "Gripper clear-fault-and-open requested."
        return future

    def request_gripper_set_protection(
        self, over_current_ma, over_temperature_c, recovery_temperature_c, save_to_flash, done_cb
    ):
        if not self.ready_for_gripper_set_protection():
            self.status = "Gripper protection writes are disabled or unavailable."
            return None
        if save_to_flash and not self.enable_gripper_flash_save:
            self.status = "Gripper Flash save is disabled."
            return None

        request = SetGripperProtection.Request()
        request.over_current_ma = int(over_current_ma)
        request.over_temperature_c = float(over_temperature_c)
        request.recovery_temperature_c = float(recovery_temperature_c)
        request.save_to_flash = bool(save_to_flash)
        future = self.gripper_set_protection_client.call_async(request)
        future.add_done_callback(done_cb)
        self.status = "Gripper protection update requested."
        return future

    def request_gripper_save_parameters(self, done_cb):
        if not self.ready_for_gripper_save_parameters():
            self.status = "Gripper Flash save is disabled or unavailable."
            return None
        future = self.gripper_save_parameters_client.call_async(Trigger.Request())
        future.add_done_callback(done_cb)
        self.status = "Gripper Flash save requested."
        return future

    def publish_initial_pose(self):
        point = JointTrajectoryPoint()
        point.positions = list(self.initial_pose_positions)
        point.time_from_start = Duration(seconds=self.initial_pose_duration_s).to_msg()

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.initial_pose_joints)
        msg.points = [point]
        self.initial_pose_pub.publish(msg)

        positions = ", ".join(f"{value:.3f}" for value in self.initial_pose_positions)
        self.status = (
            f"Published initial pose [{positions}] to {self.initial_pose_topic} "
            f"over {self.initial_pose_duration_s:.1f}s."
        )
        self.get_logger().warn(self.status)

    def request_emergency_gripper_open(self):
        if not self.enable_emergency_gripper_open:
            self.status = "Emergency gripper open is disabled."
            return
        if self.gripper_action_client is None or not self.gripper_action_client.server_is_ready():
            self.status = (
                f"Gripper action server {self.gripper_action_name} is not available."
            )
            self.get_logger().warn(self.status)
            return

        self.status = "Emergency gripper open requested."
        self.get_logger().warn(self.status)

        if (
            self.gripper_cancel_before_open
            and self.gripper_cancel_client is not None
            and self.gripper_cancel_client.service_is_ready()
        ):
            request = CancelGoal.Request()
            future = self.gripper_cancel_client.call_async(request)
            future.add_done_callback(self._on_emergency_cancel_done)
            return

        if self.gripper_cancel_before_open:
            self.get_logger().warn(
                "Gripper cancel service is not available; sending open goal directly."
            )
        self._send_emergency_gripper_open_goal()

    def _on_emergency_cancel_done(self, future):
        try:
            response = future.result()
            self.get_logger().warn(
                "Emergency gripper cancel requested; return_code=%d, goals_canceling=%d",
                response.return_code,
                len(response.goals_canceling),
            )
        except Exception as exc:
            self.get_logger().warn(f"Emergency gripper cancel request failed: {exc}")

        self._send_emergency_gripper_open_goal()

    def _send_emergency_gripper_open_goal(self):
        if self.gripper_action_client is None or not self.gripper_action_client.server_is_ready():
            self.status = (
                f"Gripper action server {self.gripper_action_name} disappeared before open goal."
            )
            self.get_logger().warn(self.status)
            return

        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_open_position
        goal.command.max_effort = self.gripper_max_effort
        future = self.gripper_action_client.send_goal_async(goal)
        future.add_done_callback(self._on_emergency_open_goal_response)
        self.status = (
            f"Emergency gripper open goal sent to {self.gripper_action_name} "
            f"at position {self.gripper_open_position:.3f}."
        )
        self.get_logger().warn(self.status)

    def _on_emergency_open_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.status = f"Emergency gripper open goal failed: {exc}"
            self.get_logger().warn(self.status)
            return

        if not goal_handle.accepted:
            self.status = "Emergency gripper open goal was rejected."
            self.get_logger().warn(self.status)
            return

        self.status = "Emergency gripper open goal accepted."
        self.get_logger().warn(self.status)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_emergency_open_result)

    def _on_emergency_open_result(self, future):
        try:
            result = future.result()
            self.status = f"Emergency gripper open finished with status {result.status}."
        except Exception as exc:
            self.status = f"Emergency gripper open result failed: {exc}"
        self.get_logger().warn(self.status)

    @staticmethod
    def _parse_string_list(value):
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return [str(item) for item in value]

    @staticmethod
    def _parse_float_list(value):
        if isinstance(value, str):
            return [float(item.strip()) for item in value.split(",") if item.strip()]
        return [float(item) for item in value]


class HomingButtonApp:
    def __init__(self, node):
        self.node = node
        self.closed = False
        self.shutdown_requested = False
        self.after_id = None
        self.last_gripper_poll_time = 0.0
        self.gripper_status_future = None
        self.gripper_backend = "unknown"
        self.gripper_recovery_available = True
        self.pending_futures = set()

        self.root = tk.Tk()
        self.root.title("Arm Operator Controls")
        self.root.geometry("520x760")
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self.request_shutdown)
        signal.signal(signal.SIGTERM, self.request_shutdown)

        self.status_var = tk.StringVar(value=self.node.status)
        self.stm32_state_var = tk.StringVar(value="STM32: waiting for feedback")
        self.initial_pose_button_var = tk.StringVar(value="Move Initial Pose")
        self.drop_button_var = tk.StringVar(value="Confirm Drop Pose / Zero Joint 3")
        self.emergency_arm_button_var = tk.StringVar(value="Emergency Stop Arm")
        self.emergency_gripper_button_var = tk.StringVar(value="Emergency Open Gripper")
        self.gripper_status_var = tk.StringVar(value=self.node.gripper_status_text)
        self.gripper_fault_button_var = tk.StringVar(value="Clear Gripper Fault")
        self.gripper_fault_open_button_var = tk.StringVar(value="Clear Fault + Open")
        self.gripper_fetch_button_var = tk.StringVar(value="Load Protection")
        self.gripper_apply_button_var = tk.StringVar(value="Apply Protection to RAM")
        self.gripper_save_button_var = tk.StringVar(value="Save Protection to Flash")
        self.gripper_current_var = tk.StringVar(value="")
        self.gripper_overtemp_var = tk.StringVar(value="")
        self.gripper_recovery_var = tk.StringVar(value="")

        frame = tk.Frame(self.root, padx=18, pady=16)
        frame.pack(fill=tk.BOTH, expand=True)

        title = tk.Label(frame, text="Arm operator controls", font=("Sans", 15, "bold"))
        title.pack(anchor="w")

        status = tk.Label(
            frame,
            textvariable=self.status_var,
            justify=tk.LEFT,
            anchor="w",
            wraplength=420,
        )
        status.pack(anchor="w", fill=tk.X, pady=(10, 14))

        marker_frame = tk.Frame(frame)
        marker_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Label(marker_frame, text="STM32 state", width=14, anchor="w").pack(side=tk.LEFT)
        self.stm32_state_label = tk.Label(
            marker_frame,
            textvariable=self.stm32_state_var,
            anchor="center",
            relief=tk.GROOVE,
            padx=8,
            pady=7,
            bg="#d7d7d7",
        )
        self.stm32_state_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.initial_pose_button = tk.Button(
            frame,
            textvariable=self.initial_pose_button_var,
            height=2,
            command=self.on_initial_pose_clicked,
        )
        self.initial_pose_button.pack(fill=tk.X, pady=(0, 8))

        self.drop_button = tk.Button(
            frame,
            textvariable=self.drop_button_var,
            height=2,
            command=self.on_confirm_clicked,
        )
        self.drop_button.pack(fill=tk.X)

        self.emergency_arm_button = tk.Button(
            frame,
            textvariable=self.emergency_arm_button_var,
            height=2,
            command=self.on_emergency_arm_stop_clicked,
            bg="#8c1d18",
            fg="white",
            activebackground="#641410",
            activeforeground="white",
        )
        self.emergency_arm_button.pack(fill=tk.X, pady=(12, 8))

        self.emergency_gripper_button = tk.Button(
            frame,
            textvariable=self.emergency_gripper_button_var,
            height=2,
            command=self.on_emergency_gripper_open_clicked,
            bg="#b3261e",
            fg="white",
            activebackground="#8c1d18",
            activeforeground="white",
        )
        self.emergency_gripper_button.pack(fill=tk.X)

        self.gripper_frame = tk.LabelFrame(frame, text="Gripper", padx=10, pady=10)
        self.gripper_frame.pack(fill=tk.X, pady=(12, 0))

        self.gripper_status_label = tk.Label(
            self.gripper_frame,
            textvariable=self.gripper_status_var,
            justify=tk.LEFT,
            anchor="w",
            wraplength=460,
        )
        self.gripper_status_label.pack(fill=tk.X)

        gripper_button_row = tk.Frame(self.gripper_frame)
        gripper_button_row.pack(fill=tk.X, pady=(8, 8))
        self.gripper_fault_button = tk.Button(
            gripper_button_row,
            textvariable=self.gripper_fault_button_var,
            command=self.on_gripper_clear_fault_clicked,
        )
        self.gripper_fault_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.gripper_fault_open_button = tk.Button(
            gripper_button_row,
            textvariable=self.gripper_fault_open_button_var,
            command=self.on_gripper_clear_fault_open_clicked,
            bg="#b3261e",
            fg="white",
            activebackground="#8c1d18",
            activeforeground="white",
        )
        self.gripper_fault_open_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))

        protection_grid = tk.Frame(self.gripper_frame)
        protection_grid.pack(fill=tk.X, pady=(4, 8))
        tk.Label(protection_grid, text="Over-current mA", anchor="w").grid(
            row=0, column=0, sticky="w", pady=2
        )
        tk.Label(protection_grid, text="Over-temp C", anchor="w").grid(
            row=1, column=0, sticky="w", pady=2
        )
        tk.Label(protection_grid, text="Recovery C", anchor="w").grid(
            row=2, column=0, sticky="w", pady=2
        )
        self.gripper_current_entry = tk.Entry(
            protection_grid, textvariable=self.gripper_current_var, width=12
        )
        self.gripper_overtemp_entry = tk.Entry(
            protection_grid, textvariable=self.gripper_overtemp_var, width=12
        )
        self.gripper_recovery_entry = tk.Entry(
            protection_grid, textvariable=self.gripper_recovery_var, width=12
        )
        self.gripper_current_entry.grid(row=0, column=1, sticky="ew", pady=2)
        self.gripper_overtemp_entry.grid(row=1, column=1, sticky="ew", pady=2)
        self.gripper_recovery_entry.grid(row=2, column=1, sticky="ew", pady=2)
        protection_grid.columnconfigure(1, weight=1)

        protection_button_row = tk.Frame(self.gripper_frame)
        protection_button_row.pack(fill=tk.X)
        self.gripper_fetch_button = tk.Button(
            protection_button_row,
            textvariable=self.gripper_fetch_button_var,
            command=self.on_gripper_fetch_protection_clicked,
        )
        self.gripper_fetch_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self.gripper_apply_button = tk.Button(
            protection_button_row,
            textvariable=self.gripper_apply_button_var,
            command=lambda: self.on_gripper_apply_protection_clicked(False),
        )
        self.gripper_apply_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.gripper_save_button = tk.Button(
            protection_button_row,
            textvariable=self.gripper_save_button_var,
            command=self.on_gripper_save_parameters_clicked,
        )
        self.gripper_save_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

        if not self.node.enable_gripper_maintenance_ui:
            self.gripper_frame.pack_forget()

        self.after_id = self.root.after(50, self.tick)

    def request_shutdown(self, *_):
        self.shutdown_requested = True

    def tick(self):
        if self.shutdown_requested or not rclpy.ok():
            self.close()
            return

        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except (KeyboardInterrupt, ExternalShutdownException):
            self.close()
            return

        self.status_var.set(self.node.status)
        self.update_stm32_marker()
        self.maybe_poll_gripper_status()
        self.gripper_status_var.set(self.node.gripper_status_text)
        self.initial_pose_button.configure(
            state=tk.NORMAL
            if self.node.ready_for_initial_pose() and not self.pending_futures
            else tk.DISABLED
        )
        self.drop_button.configure(
            state=tk.NORMAL
            if self.node.ready_for_drop_confirmation() and not self.pending_futures
            else tk.DISABLED
        )
        self.emergency_arm_button.configure(
            state=tk.NORMAL
            if self.node.ready_for_emergency_arm_stop()
            else tk.DISABLED
        )
        self.emergency_gripper_button.configure(
            state=tk.NORMAL
            if self.node.ready_for_emergency_gripper_open()
            else tk.DISABLED
        )
        if self.node.enable_gripper_maintenance_ui:
            self.gripper_fault_button.configure(
                state=tk.NORMAL
                if self.node.ready_for_gripper_clear_fault()
                else tk.DISABLED
            )
            self.gripper_fault_open_button.configure(
                state=tk.NORMAL
                if self.node.ready_for_gripper_clear_fault_open()
                else tk.DISABLED
            )
            self.gripper_fetch_button.configure(
                state=tk.NORMAL
                if self.node.ready_for_gripper_get_protection()
                else tk.DISABLED
            )
            protection_write_state = (
                tk.NORMAL if self.node.ready_for_gripper_set_protection() else tk.DISABLED
            )
            self.gripper_apply_button.configure(state=protection_write_state)
            self.gripper_current_entry.configure(state=protection_write_state)
            self.gripper_overtemp_entry.configure(state=protection_write_state)
            self.gripper_recovery_entry.configure(
                state=protection_write_state
                if self.gripper_recovery_available
                else tk.DISABLED
            )
            self.gripper_save_button.configure(
                state=tk.NORMAL
                if self.node.ready_for_gripper_save_parameters()
                and self.gripper_backend != "dynamixel_xw430"
                else tk.DISABLED
            )
        self.after_id = self.root.after(50, self.tick)

    def update_stm32_marker(self):
        state = self.node.stm32_state
        if state is None:
            text = "STM32: no feedback"
            color = "#d7d7d7"
        else:
            label = STM32_STATE_LABELS.get(state, f"UNKNOWN({state})")
            text = f"STM32: {label}"
            color = STM32_STATE_COLORS.get(state, "#f4d99b")
        self.stm32_state_var.set(text)
        self.stm32_state_label.configure(bg=color)

    def maybe_poll_gripper_status(self):
        if not self.node.enable_gripper_maintenance_ui:
            return
        now = self.node.get_clock().now().nanoseconds / 1e9
        if self.gripper_status_future is not None and not self.gripper_status_future.done():
            return
        if now - self.last_gripper_poll_time < self.node.gripper_status_poll_s:
            return
        self.last_gripper_poll_time = now
        future = self.node.request_gripper_status(self.on_gripper_status_done)
        if future is not None:
            self.gripper_status_future = future

    def on_gripper_status_done(self, future):
        self.gripper_status_future = None
        try:
            response = future.result()
            if not response.success:
                self.node.gripper_status_text = f"Gripper: {response.message}"
                return
            self.gripper_backend = response.backend
            faults = []
            if response.stall_protection:
                faults.append("stall")
            if response.over_temperature:
                faults.append("over-temp")
            if response.over_current:
                faults.append("over-current")
            if response.motor_abnormal:
                faults.append("motor abnormal")
            fault_text = ", ".join(faults) if faults else "none"
            self.node.gripper_status_text = (
                f"{response.backend}: position {response.position:.3f} "
                f"({response.position_units} units), "
                f"target {response.target_units} units\n"
                f"Current {response.current_ma} mA, "
                f"temperature {response.temperature_c:.0f} C, "
                f"faults: {fault_text}"
            )
        except Exception as exc:
            self.node.gripper_status_text = f"Gripper status failed: {exc}"

    def on_initial_pose_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        try:
            self.node.publish_initial_pose()
            self.initial_pose_button_var.set("Initial Pose Sent")
            self.root.after(
                1200, lambda: self.initial_pose_button_var.set("Move Initial Pose")
            )
        except Exception as exc:
            msg = f"Failed to publish initial pose: {exc}"
            self.node.status = msg
            messagebox.showerror("Arm Homing", msg)

    def on_confirm_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        self.drop_button.configure(state=tk.DISABLED)
        self.drop_button_var.set("Sending...")
        future = self.node.confirm_drop_pose(self.on_confirm_done)
        self.pending_futures.add(future)

    def on_emergency_arm_stop_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        self.emergency_arm_button_var.set("Stopping Arm...")
        future = self.node.request_emergency_arm_stop(self.on_emergency_arm_stop_done)
        if future is not None:
            self.pending_futures.add(future)

    def on_emergency_arm_stop_done(self, future):
        self.on_service_done(
            future,
            self.emergency_arm_button_var,
            "Emergency Stop Arm",
            self.node.arm_emergency_stop_service,
        )

    def on_emergency_gripper_open_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        self.emergency_gripper_button_var.set("Opening Gripper...")
        self.node.request_emergency_gripper_open()
        self.root.after(
            1500,
            lambda: self.emergency_gripper_button_var.set("Emergency Open Gripper"),
        )

    def on_gripper_clear_fault_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        if not messagebox.askyesno(
            "Gripper Fault",
            "Clear gripper protection fault flags now?\n\n"
            "This does not clear over-temperature; that requires cooling.",
        ):
            return
        self.gripper_fault_button_var.set("Clearing...")
        future = self.node.request_gripper_clear_fault(self.on_gripper_clear_fault_done)
        if future is not None:
            self.pending_futures.add(future)

    def on_gripper_clear_fault_open_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        if not messagebox.askyesno(
            "Gripper Recovery",
            "Cancel active gripper goals, clear protection faults, and command the "
            "gripper open?",
        ):
            return
        self.gripper_fault_open_button_var.set("Recovering...")
        self.node.request_gripper_clear_fault_open(self.on_gripper_clear_fault_open_done)

    def on_gripper_fetch_protection_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        self.gripper_fetch_button_var.set("Loading...")
        future = self.node.request_gripper_get_protection(
            self.on_gripper_fetch_protection_done
        )
        if future is not None:
            self.pending_futures.add(future)

    def on_gripper_fetch_protection_done(self, future):
        if self.closed or self.shutdown_requested:
            return
        try:
            self.pending_futures.discard(future)
            response = future.result()
            self.node.status = response.message
            if not response.success:
                messagebox.showwarning("Gripper Protection", response.message)
                return
            self.gripper_recovery_available = bool(
                response.recovery_temperature_available
            )
            self.gripper_backend = response.backend
            if self.gripper_backend == "dynamixel_xw430":
                self.gripper_apply_button_var.set("Apply Protection to EEPROM")
                self.gripper_save_button_var.set("No Flash Save Needed")
            else:
                self.gripper_apply_button_var.set("Apply Protection to RAM")
                self.gripper_save_button_var.set("Save Protection to Flash")
            self.gripper_current_var.set(str(response.over_current_ma))
            self.gripper_overtemp_var.set(f"{response.over_temperature_c:.1f}")
            if self.gripper_recovery_available:
                self.gripper_recovery_var.set(f"{response.recovery_temperature_c:.1f}")
            else:
                self.gripper_recovery_var.set("N/A")
        except Exception as exc:
            msg = f"Failed to read gripper protection parameters: {exc}"
            self.node.status = msg
            messagebox.showerror("Gripper Protection", msg)
        finally:
            self.gripper_fetch_button_var.set("Load Protection")

    def on_gripper_apply_protection_clicked(self, save_to_flash):
        if self.closed or self.shutdown_requested:
            return
        try:
            over_current_ma = int(self.gripper_current_var.get().strip())
            over_temperature_c = float(self.gripper_overtemp_var.get().strip())
            recovery_temperature_c = (
                float(self.gripper_recovery_var.get().strip())
                if self.gripper_recovery_available
                else 0.0
            )
        except ValueError as exc:
            messagebox.showerror("Gripper Protection", f"Invalid protection value: {exc}")
            return

        target = (
            "EEPROM"
            if self.gripper_backend == "dynamixel_xw430"
            else ("RAM and Flash" if save_to_flash else "RAM")
        )
        if not messagebox.askyesno(
            "Gripper Protection",
            f"Apply gripper protection settings to {target}?\n\n"
            f"Over-current: {over_current_ma} mA\n"
            f"Over-temp: {over_temperature_c:.1f} C\n"
            + (
                f"Recovery: {recovery_temperature_c:.1f} C"
                if self.gripper_recovery_available
                else "Recovery: N/A for this backend"
            ),
        ):
            return

        self.gripper_apply_button_var.set("Applying...")
        future = self.node.request_gripper_set_protection(
            over_current_ma,
            over_temperature_c,
            recovery_temperature_c,
            save_to_flash,
            self.on_gripper_apply_protection_done,
        )
        if future is not None:
            self.pending_futures.add(future)

    def on_gripper_apply_protection_done(self, future):
        if self.closed or self.shutdown_requested:
            return
        try:
            self.pending_futures.discard(future)
            response = future.result()
            self.node.status = response.message
            if not response.success:
                messagebox.showwarning("Gripper Protection", response.message)
                return
            self.gripper_current_var.set(str(response.applied_over_current_ma))
            self.gripper_overtemp_var.set(f"{response.applied_over_temperature_c:.1f}")
            if self.gripper_recovery_available:
                self.gripper_recovery_var.set(f"{response.applied_recovery_temperature_c:.1f}")
            else:
                self.gripper_recovery_var.set("N/A")
        except Exception as exc:
            msg = f"Failed to apply gripper protection parameters: {exc}"
            self.node.status = msg
            messagebox.showerror("Gripper Protection", msg)
        finally:
            self.gripper_apply_button_var.set(
                "Apply Protection to EEPROM"
                if self.gripper_backend == "dynamixel_xw430"
                else "Apply Protection to RAM"
            )

    def on_gripper_save_parameters_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        if not messagebox.askyesno(
            "Save Gripper Parameters",
            "Save the actuator's current RAM parameters to internal Flash?\n\n"
            "This survives power cycling.",
        ):
            return
        self.gripper_save_button_var.set("Saving...")
        future = self.node.request_gripper_save_parameters(
            self.on_gripper_save_parameters_done
        )
        if future is not None:
            self.pending_futures.add(future)

    def on_gripper_save_parameters_done(self, future):
        self.on_service_done(
            future,
            self.gripper_save_button_var,
            "Save Protection to Flash",
            self.node.gripper_save_parameters_service,
        )

    def on_gripper_clear_fault_done(self, future):
        self.on_service_done(
            future,
            self.gripper_fault_button_var,
            "Clear Gripper Fault",
            self.node.gripper_clear_fault_service,
        )

    def on_gripper_clear_fault_open_done(self, future):
        self.on_service_done(
            future,
            self.gripper_fault_open_button_var,
            "Clear Fault + Open",
            self.node.gripper_clear_fault_open_service,
        )

    def on_confirm_done(self, future):
        self.on_service_done(
            future,
            self.drop_button_var,
            "Confirm Drop Pose / Zero Joint 3",
            "/arm_homing/confirm_drop_pose",
        )

    def on_service_done(self, future, button_var, idle_label, service_name):
        if self.closed or self.shutdown_requested:
            return

        try:
            self.pending_futures.discard(future)
            response = future.result()
            self.node.status = response.message
            if not response.success:
                self.root.after(
                    0, lambda: messagebox.showwarning("Arm Homing", response.message)
                )
        except Exception as exc:
            msg = f"Failed to call {service_name}: {exc}"
            self.node.status = msg
            self.root.after(0, lambda: messagebox.showerror("Arm Homing", msg))
        finally:
            if not self.closed and not self.shutdown_requested:
                self.root.after(0, lambda: button_var.set(idle_label))

    def close(self):
        if self.closed:
            return

        self.closed = True
        self.shutdown_requested = True
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except tk.TclError:
                pass
            self.after_id = None

        for future in list(self.pending_futures):
            future.cancel()
        self.pending_futures.clear()

        try:
            self.root.quit()
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.close()


def main():
    rclpy.init()
    node = HomingButtonNode()
    app = None
    try:
        app = HomingButtonApp(node)
        app.run()
    except KeyboardInterrupt:
        if app is not None:
            app.close()
    finally:
        if app is not None:
            app.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
