#!/usr/bin/env python3

import math
import select
import termios
import threading
import time
import tty
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clamp_unit(value: float) -> float:
    return max(0.0, min(1.0, value))


def clamp_vector(values: List[float], max_norm: float) -> List[float]:
    if max_norm <= 0.0:
        return [0.0 for _ in values]
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= max_norm or norm <= 1e-12:
        return values
    scale = max_norm / norm
    return [value * scale for value in values]


def normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def rotate_vector_by_quaternion(
    values: List[float], q: Tuple[float, float, float, float]
) -> List[float]:
    x, y, z, w = normalize_quaternion(q)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    vx, vy, vz = values
    return [
        (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz,
    ]


class KeyBackend:
    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def pressed(self) -> Set[str]:
        return set()

    def consume_pulses(self) -> Set[str]:
        return set()


class EvdevBackend(KeyBackend):
    KEY_ALIASES = {
        "space": "KEY_SPACE",
        "esc": "KEY_ESC",
        "escape": "KEY_ESC",
        "leftshift": "KEY_LEFTSHIFT",
        "rightshift": "KEY_RIGHTSHIFT",
        "shift": "KEY_LEFTSHIFT",
        "leftctrl": "KEY_LEFTCTRL",
        "rightctrl": "KEY_RIGHTCTRL",
        "ctrl": "KEY_LEFTCTRL",
    }

    def __init__(
        self,
        node: Node,
        device_path: str,
        device_name: str,
        grab_device: bool,
        all_keys: Set[str],
    ):
        self.node = node
        self.device_path = device_path.strip()
        self.device_name = device_name.strip().lower()
        self.grab_device = grab_device
        self.all_keys = all_keys
        self.device = None
        self.key_codes: Dict[int, str] = {}
        self.pressed_keys: Set[str] = set()
        self.pulse_keys: Set[str] = set()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        try:
            import evdev
        except ImportError as exc:
            raise RuntimeError(
                "keyboard_backend:=evdev requires python3-evdev. Install it with "
                "`sudo apt install python3-evdev` and add your user to the input group."
            ) from exc

        self.evdev = evdev
        self.device = self._open_device()
        self.key_codes = {
            self._key_to_code(key): key for key in self.all_keys if self._key_to_code(key) is not None
        }
        if not self.key_codes:
            raise RuntimeError("No valid evdev key codes were configured.")

        if self.grab_device:
            try:
                self.device.grab()
                self.node.get_logger().warn(f"Grabbed keyboard input device: {self.device.path}")
            except OSError:
                self.node.get_logger().warn(
                    f"Using keyboard input device without exclusive grab: {self.device.path}"
                )
        else:
            self.node.get_logger().warn(
                f"Using keyboard input device without exclusive grab: {self.device.path}"
            )

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.device is not None:
            try:
                self.device.ungrab()
            except OSError:
                pass
            try:
                self.device.close()
            except OSError:
                pass

    def pressed(self) -> Set[str]:
        with self.lock:
            return set(self.pressed_keys)

    def consume_pulses(self) -> Set[str]:
        with self.lock:
            pulses = set(self.pulse_keys)
            self.pulse_keys.clear()
            return pulses

    def _open_device(self):
        if self.device_path:
            return self.evdev.InputDevice(self.device_path)

        candidates = []
        for path in self.evdev.list_devices():
            try:
                device = self.evdev.InputDevice(path)
                caps = device.capabilities()
                key_caps = caps.get(self.evdev.ecodes.EV_KEY, [])
                if self._key_to_code("space") not in key_caps:
                    device.close()
                    continue
                candidates.append(device)
            except OSError:
                continue

        if self.device_name:
            for device in candidates:
                if self.device_name in device.name.lower():
                    return device

        for device in candidates:
            name = device.name.lower()
            if "keyboard" in name or "keychron" in name or "thinkpad" in name:
                return device

        if candidates:
            return candidates[0]

        raise RuntimeError(
            "No readable keyboard input device found. Try `sudo usermod -aG input $USER`, "
            "log out/in, or set keyboard_device_path:=/dev/input/eventX."
        )

    def _key_to_code(self, key: str) -> Optional[int]:
        token = key.strip().lower()
        code_name = self.KEY_ALIASES.get(token)
        if code_name is None:
            if len(token) == 1 and token.isalnum():
                code_name = f"KEY_{token.upper()}"
            else:
                code_name = f"KEY_{token.upper()}"
        return getattr(self.evdev.ecodes, code_name, None)

    def _run(self) -> None:
        assert self.device is not None
        try:
            for event in self.device.read_loop():
                if self.stop_event.is_set():
                    return
                if event.type != self.evdev.ecodes.EV_KEY or event.code not in self.key_codes:
                    continue
                key = self.key_codes[event.code]
                with self.lock:
                    if event.value == 0:
                        self.pressed_keys.discard(key)
                    elif event.value == 1:
                        self.pressed_keys.add(key)
                        self.pulse_keys.add(key)
                    elif event.value == 2:
                        self.pressed_keys.add(key)
        except OSError as exc:
            if not self.stop_event.is_set():
                self.node.get_logger().error(f"Keyboard input device stopped: {exc}")


class TerminalBackend(KeyBackend):
    def __init__(self, node: Node, all_keys: Set[str], key_hold_s: float):
        self.node = node
        self.all_keys = all_keys
        self.key_hold_s = max(float(key_hold_s), 0.05)
        self.original_term = None
        self.tty_file = None
        self.last_seen: Dict[str, float] = {}
        self.pulse_keys: Set[str] = set()

    def start(self) -> None:
        try:
            self.tty_file = open("/dev/tty", "r", buffering=1)
        except OSError as exc:
            raise RuntimeError(
                "keyboard_backend:=terminal requires an interactive controlling terminal."
            ) from exc
        self.original_term = termios.tcgetattr(self.tty_file)
        tty.setcbreak(self.tty_file.fileno())
        self.node.get_logger().warn(
            "Using terminal keyboard backend. It cannot see true key-release events; "
            "evdev is preferred for real deadman behavior."
        )

    def stop(self) -> None:
        if self.original_term is not None and self.tty_file is not None:
            termios.tcsetattr(self.tty_file, termios.TCSADRAIN, self.original_term)
        if self.tty_file is not None:
            self.tty_file.close()

    def pressed(self) -> Set[str]:
        self._read_available()
        now = time.monotonic()
        return {
            key for key, stamp in self.last_seen.items() if now - stamp <= self.key_hold_s
        }

    def consume_pulses(self) -> Set[str]:
        self._read_available()
        pulses = set(self.pulse_keys)
        self.pulse_keys.clear()
        return pulses

    def _read_available(self) -> None:
        if self.tty_file is None:
            return
        while True:
            ready, _, _ = select.select([self.tty_file], [], [], 0.0)
            if not ready:
                return
            char = self.tty_file.read(1)
            key = self._char_to_key(char)
            if key in self.all_keys:
                self.last_seen[key] = time.monotonic()
                self.pulse_keys.add(key)

    @staticmethod
    def _char_to_key(char: str) -> str:
        if char == " ":
            return "space"
        if char == "\x1b":
            return "esc"
        if char == "\x03":
            raise KeyboardInterrupt
        return char.lower()


class KeyboardTwistTeleop(Node):
    """Keyboard fallback that mimics the Geomagic raw twist/buttons contract."""

    def __init__(self):
        super().__init__("keyboard_twist_teleop")

        self.declare_parameter("keyboard_backend", "evdev")
        self.declare_parameter("keyboard_device_path", "")
        self.declare_parameter("keyboard_device_name", "")
        self.declare_parameter("keyboard_grab_device", False)
        self.declare_parameter("output_twist_topic", "/arm_teleop/raw_twist_cmd")
        self.declare_parameter("output_buttons_topic", "/geomagic_touch/buttons")
        self.declare_parameter("status_topic", "/arm_teleop/keyboard_status")
        self.declare_parameter("command_frame", "uav_mount")
        self.declare_parameter("angular_command_frame", "tool0")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("linear_speed_m_s", 0.04)
        self.declare_parameter("angular_speed_rad_s", 0.20)
        self.declare_parameter("fast_scale", 2.0)
        self.declare_parameter("slow_scale", 0.35)
        self.declare_parameter("low_pass_alpha", 0.35)
        self.declare_parameter("terminal_key_hold_s", 0.18)
        self.declare_parameter("deadman_key", "space")
        self.declare_parameter("gripper_key", "g")
        self.declare_parameter("positive_x_key", "w")
        self.declare_parameter("negative_x_key", "s")
        self.declare_parameter("positive_y_key", "a")
        self.declare_parameter("negative_y_key", "d")
        self.declare_parameter("positive_z_key", "z")
        self.declare_parameter("negative_z_key", "x")
        self.declare_parameter("positive_tool_z_key", "q")
        self.declare_parameter("negative_tool_z_key", "e")
        self.declare_parameter("fast_key", "leftshift")
        self.declare_parameter("slow_key", "leftctrl")
        self.declare_parameter("quit_key", "esc")
        self.declare_parameter("gripper_pulse_s", 0.15)

        self.backend_name = self.get_parameter("keyboard_backend").value
        self.output_twist_topic = self.get_parameter("output_twist_topic").value
        self.output_buttons_topic = self.get_parameter("output_buttons_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.command_frame = self.get_parameter("command_frame").value
        self.angular_command_frame = self.get_parameter("angular_command_frame").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.linear_speed = float(self.get_parameter("linear_speed_m_s").value)
        self.angular_speed = float(self.get_parameter("angular_speed_rad_s").value)
        self.fast_scale = float(self.get_parameter("fast_scale").value)
        self.slow_scale = float(self.get_parameter("slow_scale").value)
        self.low_pass_alpha = clamp_unit(float(self.get_parameter("low_pass_alpha").value))
        self.gripper_pulse_s = float(self.get_parameter("gripper_pulse_s").value)

        self.deadman_key = self._key_param("deadman_key")
        self.gripper_key = self._key_param("gripper_key")
        self.positive_x_key = self._key_param("positive_x_key")
        self.negative_x_key = self._key_param("negative_x_key")
        self.positive_y_key = self._key_param("positive_y_key")
        self.negative_y_key = self._key_param("negative_y_key")
        self.positive_z_key = self._key_param("positive_z_key")
        self.negative_z_key = self._key_param("negative_z_key")
        self.positive_tool_z_key = self._key_param("positive_tool_z_key")
        self.negative_tool_z_key = self._key_param("negative_tool_z_key")
        self.fast_key = self._key_param("fast_key")
        self.slow_key = self._key_param("slow_key")
        self.quit_key = self._key_param("quit_key")

        self.all_keys = {
            self.deadman_key,
            self.gripper_key,
            self.positive_x_key,
            self.negative_x_key,
            self.positive_y_key,
            self.negative_y_key,
            self.positive_z_key,
            self.negative_z_key,
            self.positive_tool_z_key,
            self.negative_tool_z_key,
            self.fast_key,
            self.slow_key,
            self.quit_key,
        }
        self.all_keys.discard("")

        self.backend = self._make_backend()
        self.backend.start()

        self.filtered_linear = [0.0, 0.0, 0.0]
        self.filtered_angular = [0.0, 0.0, 0.0]
        self.gripper_pulse_until = self.get_clock().now() - Duration(seconds=1.0)
        self.last_tf_warn_time = self.get_clock().now() - Duration(seconds=10.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_pub = self.create_publisher(TwistStamped, self.output_twist_topic, 10)
        self.buttons_pub = self.create_publisher(Joy, self.output_buttons_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().warn(
            "Keyboard teleop ready: hold "
            f"{self.deadman_key}, move {self.positive_x_key}/{self.negative_x_key} "
            f"{self.positive_y_key}/{self.negative_y_key} "
            f"{self.positive_z_key}/{self.negative_z_key}, tool-Z "
            f"{self.positive_tool_z_key}/{self.negative_tool_z_key}, gripper={self.gripper_key}"
        )

    def destroy_node(self):
        try:
            if rclpy.ok():
                self.publish_zero(deadman=False)
            self.backend.stop()
        finally:
            super().destroy_node()

    def _key_param(self, name: str) -> str:
        return str(self.get_parameter(name).value).strip().lower()

    def _make_backend(self) -> KeyBackend:
        backend = str(self.backend_name).strip().lower()
        if backend == "terminal":
            return TerminalBackend(
                self,
                self.all_keys,
                float(self.get_parameter("terminal_key_hold_s").value),
            )
        if backend != "evdev":
            raise RuntimeError("keyboard_backend must be 'evdev' or 'terminal'.")
        return EvdevBackend(
            self,
            self.get_parameter("keyboard_device_path").value,
            self.get_parameter("keyboard_device_name").value,
            as_bool(self.get_parameter("keyboard_grab_device").value),
            self.all_keys,
        )

    def timer_callback(self) -> None:
        pressed = self.backend.pressed()
        pulses = self.backend.consume_pulses()
        if self.quit_key and self.quit_key in pulses:
            self.get_logger().warn("Quit key pressed; publishing zero command.")
            self.publish_zero(deadman=False)
            rclpy.shutdown()
            return
        if self.gripper_key and self.gripper_key in pulses:
            self.gripper_pulse_until = self.get_clock().now() + Duration(
                seconds=max(self.gripper_pulse_s, 0.01)
            )

        deadman = self.deadman_key in pressed
        linear = [0.0, 0.0, 0.0]
        angular = [0.0, 0.0, 0.0]
        if deadman:
            scale = 1.0
            if self.fast_key in pressed:
                scale *= self.fast_scale
            if self.slow_key in pressed:
                scale *= self.slow_scale
            linear_speed = self.linear_speed * scale
            angular_speed = self.angular_speed * scale

            linear[0] = self._axis(pressed, self.positive_x_key, self.negative_x_key) * linear_speed
            linear[1] = self._axis(pressed, self.positive_y_key, self.negative_y_key) * linear_speed
            linear[2] = self._axis(pressed, self.positive_z_key, self.negative_z_key) * linear_speed
            linear = clamp_vector(linear, abs(linear_speed))

            tool_z = self._axis(pressed, self.positive_tool_z_key, self.negative_tool_z_key)
            if abs(tool_z) > 0.0:
                angular = self.tool_z_axis_in_command_frame(tool_z * angular_speed)
            angular = clamp_vector(angular, abs(angular_speed))

        self.filtered_linear = self._smooth(self.filtered_linear, linear)
        self.filtered_angular = self._smooth(self.filtered_angular, angular)
        self.publish_command(deadman, self.filtered_linear, self.filtered_angular)

    @staticmethod
    def _axis(pressed: Set[str], positive_key: str, negative_key: str) -> float:
        return (1.0 if positive_key in pressed else 0.0) - (
            1.0 if negative_key in pressed else 0.0
        )

    def _smooth(self, previous: List[float], target: List[float]) -> List[float]:
        alpha = self.low_pass_alpha
        return [
            previous[index] + alpha * (target[index] - previous[index])
            for index in range(3)
        ]

    def tool_z_axis_in_command_frame(self, speed: float) -> List[float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.command_frame,
                self.angular_command_frame,
                Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException as exc:
            now = self.get_clock().now()
            if (now - self.last_tf_warn_time) > Duration(seconds=2.0):
                self.get_logger().warn(
                    "Cannot transform keyboard tool-Z angular command from "
                    f"{self.angular_command_frame} to {self.command_frame}: {exc}"
                )
                self.last_tf_warn_time = now
            return [0.0, 0.0, speed]

        q = transform.transform.rotation
        return rotate_vector_by_quaternion([0.0, 0.0, speed], (q.x, q.y, q.z, q.w))

    def publish_command(self, deadman: bool, linear: List[float], angular: List[float]) -> None:
        now = self.get_clock().now()

        buttons = Joy()
        buttons.header.stamp = now.to_msg()
        buttons.buttons = [
            1 if deadman else 0,
            1 if now < self.gripper_pulse_until else 0,
        ]
        self.buttons_pub.publish(buttons)

        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = self.command_frame
        twist.twist.linear.x = linear[0]
        twist.twist.linear.y = linear[1]
        twist.twist.linear.z = linear[2]
        twist.twist.angular.x = angular[0]
        twist.twist.angular.y = angular[1]
        twist.twist.angular.z = angular[2]
        self.twist_pub.publish(twist)

    def publish_zero(self, deadman: bool) -> None:
        self.publish_command(deadman, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def publish_status(self) -> None:
        pressed = sorted(self.backend.pressed())
        msg = String()
        msg.data = (
            f"backend={self.backend_name} pressed={pressed} "
            f"linear_speed={self.linear_speed:.3f} angular_speed={self.angular_speed:.3f}"
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = KeyboardTwistTeleop()
    except (RuntimeError, OSError) as exc:
        node = Node("keyboard_twist_teleop_error")
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return

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
