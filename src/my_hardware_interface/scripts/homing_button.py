#!/usr/bin/env python3

import signal
import tkinter as tk
from tkinter import messagebox

import rclpy
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
        self.initial_pose_topic = self.get_parameter("initial_pose_trajectory_topic").value
        self.initial_pose_joints = self._parse_string_list(
            self.get_parameter("initial_pose_joints").value
        )
        self.initial_pose_positions = self._parse_float_list(
            self.get_parameter("initial_pose_positions").value
        )
        self.initial_pose_duration_s = float(self.get_parameter("initial_pose_duration_s").value)
        if len(self.initial_pose_positions) != len(self.initial_pose_joints):
            self.get_logger().error(
                "initial_pose_positions has %d values but initial_pose_joints has %d; "
                "Move Initial Pose will stay disabled.",
                len(self.initial_pose_positions),
                len(self.initial_pose_joints),
            )
            self.initial_pose_positions = []

        self.drop_client = self.create_client(Trigger, "/arm_homing/confirm_drop_pose")
        self.initial_pose_pub = self.create_publisher(
            JointTrajectory, self.initial_pose_topic, 10
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

    def confirm_drop_pose(self, done_cb):
        request = Trigger.Request()
        future = self.drop_client.call_async(request)
        future.add_done_callback(done_cb)
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
        self.pending_futures = set()

        self.root = tk.Tk()
        self.root.title("Arm Homing")
        self.root.geometry("460x330")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self.request_shutdown)
        signal.signal(signal.SIGTERM, self.request_shutdown)

        self.status_var = tk.StringVar(value=self.node.status)
        self.stm32_state_var = tk.StringVar(value="STM32: waiting for feedback")
        self.initial_pose_button_var = tk.StringVar(value="Move Initial Pose")
        self.drop_button_var = tk.StringVar(value="Confirm Drop Pose / Zero Joint 3")

        frame = tk.Frame(self.root, padx=18, pady=16)
        frame.pack(fill=tk.BOTH, expand=True)

        title = tk.Label(frame, text="STM32 arm startup", font=("Sans", 15, "bold"))
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
