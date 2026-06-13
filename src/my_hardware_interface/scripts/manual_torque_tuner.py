#!/usr/bin/env python3

import signal
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def parse_csv(value, count, default):
    if isinstance(value, (list, tuple)):
        items = list(value)
    else:
        text = str(value).strip()
        if text.startswith("["):
            text = text[1:]
        if text.endswith("]"):
            text = text[:-1]
        items = [item.strip() for item in text.split(",") if item.strip()]

    values = []
    for item in items[:count]:
        try:
            values.append(float(item))
        except (TypeError, ValueError):
            values.append(default)
    while len(values) < count:
        values.append(default)
    return values


class ManualTorqueTunerNode(Node):
    def __init__(self):
        super().__init__("manual_torque_tuner")
        self.declare_parameter("output_topic", "/arm_dynamics/torques_nm")
        self.declare_parameter("joint_names", "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6")
        self.declare_parameter("initial_torques_nm", "0,0,0,0,0,0")
        self.declare_parameter("torque_abs_max_nm", "10,10,10,10,10,10")
        self.declare_parameter("live_update", False)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.output_topic = str(self.get_parameter("output_topic").value)
        joint_text = str(self.get_parameter("joint_names").value)
        self.joint_names = [item.strip() for item in joint_text.split(",") if item.strip()]
        if not self.joint_names:
            self.joint_names = [f"joint_{i + 1}" for i in range(6)]
        self.joint_names = self.joint_names[:6]
        while len(self.joint_names) < 6:
            self.joint_names.append(f"joint_{len(self.joint_names) + 1}")

        self.initial_torques = parse_csv(self.get_parameter("initial_torques_nm").value, 6, 0.0)
        self.torque_abs_max = [
            max(0.0, value)
            for value in parse_csv(self.get_parameter("torque_abs_max_nm").value, 6, 10.0)
        ]
        self.live_update = as_bool(self.get_parameter("live_update").value)
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.current_torques = self.clamp_values(self.initial_torques)
        self.pub = self.create_publisher(Float64MultiArray, self.output_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_current)

    def clamp_values(self, values):
        clamped = []
        for idx, value in enumerate(values[:6]):
            limit = self.torque_abs_max[idx]
            clamped.append(max(-limit, min(limit, float(value))))
        while len(clamped) < 6:
            clamped.append(0.0)
        return clamped

    def publish_torques(self, values):
        self.current_torques = self.clamp_values(values)
        self.publish_current()

    def publish_current(self):
        msg = Float64MultiArray()
        msg.data = list(self.current_torques)
        self.pub.publish(msg)


class ManualTorqueTunerApp:
    def __init__(self, node):
        self.node = node
        self.closed = False
        self.shutdown_requested = False
        self.after_id = None
        self.pending_publish_id = None
        self.vars = []
        self.scales = []
        self.entries = []

        self.root = tk.Tk()
        self.root.title("Manual torque feedforward tuner")
        self.root.geometry("620x390")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self.request_shutdown)
        signal.signal(signal.SIGTERM, self.request_shutdown)

        self.status_var = tk.StringVar(
            value=(
                f"Publishing manual torques to {self.node.output_topic} "
                f"at {self.node.publish_rate_hz:.1f} Hz"
            )
        )
        self.live_var = tk.BooleanVar(value=self.node.live_update)

        outer = tk.Frame(self.root, padx=16, pady=14)
        outer.pack(fill=tk.BOTH, expand=True)

        title = tk.Label(outer, text="Manual torque feedforward [Nm]", font=("Sans", 15, "bold"))
        title.grid(row=0, column=0, columnspan=4, sticky="w")

        tk.Label(
            outer,
            text="Use with dynamics_feedforward_source:=topic and start_dynamics_monitor:=false.",
            anchor="w",
        ).grid(row=1, column=0, columnspan=4, sticky="ew", pady=(4, 12))

        for idx, name in enumerate(self.node.joint_names):
            row = idx + 2
            limit = self.node.torque_abs_max[idx]
            var = tk.DoubleVar(value=self.node.initial_torques[idx])
            self.vars.append(var)

            tk.Label(outer, text=name, width=8, anchor="w").grid(row=row, column=0, sticky="w")
            entry = tk.Entry(outer, textvariable=var, width=10)
            scale = tk.Scale(
                outer,
                from_=-limit,
                to=limit,
                orient=tk.HORIZONTAL,
                resolution=0.01,
                length=360,
                showvalue=False,
                variable=var,
                command=lambda _value: self.schedule_live_publish(),
            )
            tk.Label(outer, text=f"+/- {limit:.2f}", width=9, anchor="e").grid(row=row, column=3)
            entry.grid(row=row, column=1, sticky="w", padx=(0, 8), pady=3)
            scale.grid(row=row, column=2, sticky="ew", pady=3)
            entry.bind("<Return>", lambda _event: self.publish_now())
            self.entries.append(entry)
            self.scales.append(scale)

        controls = tk.Frame(outer)
        controls.grid(row=8, column=0, columnspan=4, sticky="ew", pady=(18, 0))
        tk.Button(controls, text="Apply Torques", height=2, command=self.publish_now).pack(
            side=tk.LEFT, fill=tk.X, expand=True
        )
        tk.Button(controls, text="Zero All", height=2, command=self.zero_all).pack(
            side=tk.LEFT, padx=8
        )
        tk.Checkbutton(controls, text="live", variable=self.live_var).pack(side=tk.LEFT)

        tk.Label(outer, textvariable=self.status_var, anchor="w", justify=tk.LEFT).grid(
            row=9, column=0, columnspan=4, sticky="ew", pady=(12, 0)
        )

        outer.columnconfigure(2, weight=1)
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
        self.after_id = self.root.after(50, self.tick)

    def values_from_ui(self):
        return [float(var.get()) for var in self.vars]

    def schedule_live_publish(self):
        if not self.live_var.get():
            return
        if self.pending_publish_id is not None:
            self.root.after_cancel(self.pending_publish_id)
        self.pending_publish_id = self.root.after(120, self.publish_now)

    def publish_now(self):
        self.pending_publish_id = None
        try:
            values = self.node.clamp_values(self.values_from_ui())
            self.node.publish_torques(values)
            text = ", ".join(f"{value:.3f}" for value in values)
            self.status_var.set(f"Published [{text}] Nm")
        except Exception as exc:
            messagebox.showerror("Manual torque feedforward tuner", f"Failed to publish torques: {exc}")

    def zero_all(self):
        for var in self.vars:
            var.set(0.0)
        self.publish_now()

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        if self.pending_publish_id is not None:
            self.root.after_cancel(self.pending_publish_id)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = ManualTorqueTunerNode()
    app = ManualTorqueTunerApp(node)
    app.run()


if __name__ == "__main__":
    main()
