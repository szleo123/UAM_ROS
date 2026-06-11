#!/usr/bin/env python3

import signal
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, UInt8


FULL_MIT_MODE = 2
GAIN_COMMAND_TOPIC = "/my_arm_system/stm32_mit_gains_cmd"
GAIN_CURRENT_TOPIC = "/my_arm_system/stm32_mit_gains_current"
CONTROL_MODE_TOPIC = "/my_arm_system/stm32_control_mode"


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class MitGainTunerNode(Node):
    def __init__(self):
        super().__init__("stm32_mit_gain_tuner")
        self.declare_parameter("kp_max", 1000.0)
        self.declare_parameter("kd_max", 100.0)
        self.declare_parameter("live_update", True)
        self.kp_max = float(self.get_parameter("kp_max").value)
        self.kd_max = float(self.get_parameter("kd_max").value)
        self.live_update = as_bool(self.get_parameter("live_update").value)
        self.mode = None
        self.gains = None

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(UInt8, CONTROL_MODE_TOPIC, self._on_mode, latched_qos)
        self.create_subscription(Float64MultiArray, GAIN_CURRENT_TOPIC, self._on_gains, latched_qos)
        self.gain_pub = self.create_publisher(Float64MultiArray, GAIN_COMMAND_TOPIC, 10)

    def _on_mode(self, msg):
        self.mode = msg.data

    def _on_gains(self, msg):
        if len(msg.data) >= 12:
            self.gains = list(msg.data[:12])

    def is_full_mit(self):
        return self.mode == FULL_MIT_MODE

    def publish_gains(self, values):
        msg = Float64MultiArray()
        msg.data = list(values)
        self.gain_pub.publish(msg)


class MitGainTunerApp:
    def __init__(self, node):
        self.node = node
        self.closed = False
        self.shutdown_requested = False
        self.after_id = None
        self.pending_publish_id = None
        self.updating_from_robot = False
        self.kp_vars = []
        self.kd_vars = []
        self.kp_scales = []
        self.kd_scales = []
        self.kp_entries = []
        self.kd_entries = []

        self.root = tk.Tk()
        self.root.title("STM32 FULL MIT gain tuner")
        self.root.geometry("760x430")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self.request_shutdown)
        signal.signal(signal.SIGTERM, self.request_shutdown)

        self.mode_var = tk.StringVar(value="mode: waiting")
        self.status_var = tk.StringVar(value="Waiting for hardware gains...")
        self.live_var = tk.BooleanVar(value=self.node.live_update)

        outer = tk.Frame(self.root, padx=16, pady=14)
        outer.pack(fill=tk.BOTH, expand=True)

        title = tk.Label(outer, text="STM32 FULL MIT Kp / Kd", font=("Sans", 15, "bold"))
        title.grid(row=0, column=0, columnspan=6, sticky="w")

        self.mode_label = tk.Label(
            outer,
            textvariable=self.mode_var,
            anchor="center",
            relief=tk.GROOVE,
            padx=8,
            pady=5,
            width=26,
        )
        self.mode_label.grid(row=0, column=4, columnspan=2, sticky="e")

        headers = ["joint", "Kp", "", "Kd", "", ""]
        for col, text in enumerate(headers):
            tk.Label(outer, text=text).grid(row=1, column=col, sticky="w", pady=(16, 4))

        for idx in range(6):
            row = idx + 2
            tk.Label(outer, text=f"joint_{idx + 1}", width=8, anchor="w").grid(row=row, column=0, sticky="w")

            kp_var = tk.DoubleVar(value=0.0)
            kd_var = tk.DoubleVar(value=0.0)
            self.kp_vars.append(kp_var)
            self.kd_vars.append(kd_var)

            kp_entry = tk.Entry(outer, textvariable=kp_var, width=9)
            kp_scale = tk.Scale(
                outer,
                from_=0.0,
                to=self.node.kp_max,
                orient=tk.HORIZONTAL,
                resolution=0.1,
                length=230,
                showvalue=False,
                variable=kp_var,
                command=lambda _value: self.schedule_live_publish(),
            )
            kd_entry = tk.Entry(outer, textvariable=kd_var, width=9)
            kd_scale = tk.Scale(
                outer,
                from_=0.0,
                to=self.node.kd_max,
                orient=tk.HORIZONTAL,
                resolution=0.01,
                length=230,
                showvalue=False,
                variable=kd_var,
                command=lambda _value: self.schedule_live_publish(),
            )

            kp_entry.grid(row=row, column=1, sticky="w", padx=(0, 6), pady=3)
            kp_scale.grid(row=row, column=2, sticky="ew", pady=3)
            kd_entry.grid(row=row, column=3, sticky="w", padx=(12, 6), pady=3)
            kd_scale.grid(row=row, column=4, sticky="ew", pady=3)
            kp_entry.bind("<Return>", lambda _event: self.publish_now())
            kd_entry.bind("<Return>", lambda _event: self.publish_now())

            self.kp_entries.append(kp_entry)
            self.kd_entries.append(kd_entry)
            self.kp_scales.append(kp_scale)
            self.kd_scales.append(kd_scale)

        controls = tk.Frame(outer)
        controls.grid(row=8, column=0, columnspan=6, sticky="ew", pady=(18, 0))
        self.apply_button = tk.Button(controls, text="Apply Gains", height=2, command=self.publish_now)
        self.apply_button.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.reload_button = tk.Button(controls, text="Use Current", height=2, command=self.load_current_gains)
        self.reload_button.pack(side=tk.LEFT, padx=8)
        tk.Checkbutton(controls, text="live", variable=self.live_var).pack(side=tk.LEFT)

        status = tk.Label(outer, textvariable=self.status_var, anchor="w", justify=tk.LEFT)
        status.grid(row=9, column=0, columnspan=6, sticky="ew", pady=(12, 0))

        outer.columnconfigure(2, weight=1)
        outer.columnconfigure(4, weight=1)
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

        self.update_mode_marker()
        if self.node.gains is not None and not any(var.get() for var in self.kp_vars + self.kd_vars):
            self.load_current_gains()
        self.after_id = self.root.after(50, self.tick)

    def update_mode_marker(self):
        if self.node.mode is None:
            text = "mode: no hardware"
            color = "#d7d7d7"
        elif self.node.is_full_mit():
            text = "mode: FULL MIT"
            color = "#b7e4b7"
        else:
            text = f"mode: {self.node.mode} (locked)"
            color = "#f3b3b3"

        self.mode_var.set(text)
        self.mode_label.configure(bg=color)
        state = tk.NORMAL if self.node.is_full_mit() else tk.DISABLED
        for widget in self.kp_entries + self.kd_entries + self.kp_scales + self.kd_scales:
            widget.configure(state=state)
        self.apply_button.configure(state=state)

    def load_current_gains(self):
        if not self.node.gains:
            return
        self.updating_from_robot = True
        for idx in range(6):
            self.kp_vars[idx].set(float(self.node.gains[idx]))
            self.kd_vars[idx].set(float(self.node.gains[idx + 6]))
        self.updating_from_robot = False
        self.status_var.set("Loaded current gains from hardware.")

    def values_from_ui(self):
        values = []
        for var in self.kp_vars:
            values.append(max(0.0, min(self.node.kp_max, float(var.get()))))
        for var in self.kd_vars:
            values.append(max(0.0, min(self.node.kd_max, float(var.get()))))
        return values

    def schedule_live_publish(self):
        if self.updating_from_robot or not self.live_var.get():
            return
        if not self.node.is_full_mit():
            return
        if self.pending_publish_id is not None:
            self.root.after_cancel(self.pending_publish_id)
        self.pending_publish_id = self.root.after(120, self.publish_now)

    def publish_now(self):
        self.pending_publish_id = None
        if not self.node.is_full_mit():
            self.status_var.set("Gains are locked because stm32_control_mode is not FULL MIT.")
            return
        try:
            values = self.values_from_ui()
            self.node.publish_gains(values)
            kp = ", ".join(f"{value:.1f}" for value in values[:6])
            kd = ", ".join(f"{value:.2f}" for value in values[6:])
            self.status_var.set(f"Published Kp [{kp}]  Kd [{kd}]")
        except Exception as exc:
            messagebox.showerror("STM32 MIT gain tuner", f"Failed to publish gains: {exc}")

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = MitGainTunerNode()
    app = MitGainTunerApp(node)
    app.run()


if __name__ == "__main__":
    main()
