#!/usr/bin/env python3

import signal
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger


WAITING_INIT_COMMAND = 0
WAITING_OPERATOR_DROP_POSE = 2


class HomingButtonNode(Node):
    def __init__(self):
        super().__init__("arm_homing_button")
        self.state = None
        self.status = "Waiting for homing status..."
        self.init_client = self.create_client(Trigger, "/arm_homing/start_initialization")
        self.drop_client = self.create_client(Trigger, "/arm_homing/confirm_drop_pose")
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/arm_homing/status_text", self._on_status, latched_qos
        )
        self.create_subscription(UInt8, "/arm_homing/state", self._on_state, latched_qos)

    def _on_status(self, msg):
        self.status = msg.data

    def _on_state(self, msg):
        self.state = msg.data

    def ready_for_initialization(self):
        return self.state == WAITING_INIT_COMMAND and self.init_client.service_is_ready()

    def ready_for_drop_confirmation(self):
        return self.state == WAITING_OPERATOR_DROP_POSE and self.drop_client.service_is_ready()

    def start_initialization(self, done_cb):
        request = Trigger.Request()
        future = self.init_client.call_async(request)
        future.add_done_callback(done_cb)
        return future

    def confirm_drop_pose(self, done_cb):
        request = Trigger.Request()
        future = self.drop_client.call_async(request)
        future.add_done_callback(done_cb)
        return future


class HomingButtonApp:
    def __init__(self, node):
        self.node = node
        self.closed = False
        self.shutdown_requested = False
        self.after_id = None
        self.pending_futures = set()

        self.root = tk.Tk()
        self.root.title("Arm Homing")
        self.root.geometry("460x240")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self.request_shutdown)
        signal.signal(signal.SIGTERM, self.request_shutdown)

        self.status_var = tk.StringVar(value=self.node.status)
        self.init_button_var = tk.StringVar(value="Initialize Damiao")
        self.drop_button_var = tk.StringVar(value="Confirm Drop Pose")

        frame = tk.Frame(self.root, padx=18, pady=16)
        frame.pack(fill=tk.BOTH, expand=True)

        title = tk.Label(frame, text="ROS-master homing", font=("Sans", 15, "bold"))
        title.pack(anchor="w")

        status = tk.Label(
            frame,
            textvariable=self.status_var,
            justify=tk.LEFT,
            anchor="w",
            wraplength=420,
        )
        status.pack(anchor="w", fill=tk.X, pady=(10, 14))

        self.init_button = tk.Button(
            frame,
            textvariable=self.init_button_var,
            height=2,
            command=self.on_init_clicked,
        )
        self.init_button.pack(fill=tk.X)

        self.drop_button = tk.Button(
            frame,
            textvariable=self.drop_button_var,
            height=2,
            command=self.on_confirm_clicked,
        )
        self.drop_button.pack(fill=tk.X, pady=(8, 0))

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
        self.init_button.configure(
            state=tk.NORMAL if self.node.ready_for_initialization() else tk.DISABLED
        )
        self.drop_button.configure(
            state=tk.NORMAL if self.node.ready_for_drop_confirmation() else tk.DISABLED
        )
        self.after_id = self.root.after(50, self.tick)

    def on_init_clicked(self):
        if self.closed or self.shutdown_requested:
            return
        self.init_button.configure(state=tk.DISABLED)
        self.init_button_var.set("Sending...")
        future = self.node.start_initialization(self.on_init_done)
        self.pending_futures.add(future)

    def on_init_done(self, future):
        self.on_service_done(
            future,
            self.init_button_var,
            "Initialize Damiao",
            "/arm_homing/start_initialization",
        )

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
            "Confirm Drop Pose",
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
