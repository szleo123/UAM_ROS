#!/usr/bin/env python3
import sys, termios, tty, select, math, time
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

HELP = """
Keyboard controls (hold or tap):
  Position:    W/S: +X/-X    A/D: +Y/-Y    R/F: +Z/-Z
  Orientation: U/O: +Roll/-Roll   I/K: +Pitch/-Pitch   J/L: +Yaw/-Yaw
  Misc:        SPACE: zero pose (all 0)    G: zero orientation
               P: print pose                H or ?: show help
               Q or ESC or CTRL-C: quit

Tips:
- Use --lin-step and --ang-step-deg to set increments.
- frame_id defaults to 'base_link'.
"""

def euler_to_quaternion(roll, pitch, yaw):
    """rpy->xyzw"""
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return x, y, z, w

class PoseTeleop(Node):
    def __init__(self, topic, frame_id, rate_hz, lin_step, ang_step_deg):
        super().__init__("teleop_pose_publisher")
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.frame_id = frame_id
        self.dt = 1.0 / rate_hz
        self.lin_step = lin_step
        self.ang_step = math.radians(ang_step_deg)

        # current target (mutable by keyboard)
        self.x = 0.7; self.y = 0.0; self.z = 0.5
        self.roll = 0.0; self.pitch = 90.0; self.yaw = 0.0

        # timer to publish
        self.timer = self.create_timer(self.dt, self._on_timer)

        # put stdin in raw mode
        self._orig_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print(HELP)
        self._print_pose()

    def destroy_node(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._orig_term)
        except Exception:
            pass
        return super().destroy_node()

    def _on_timer(self):
        # non-blocking key read
        while self._kbhit():
            c = sys.stdin.read(1)
            if not c:
                break
            if ord(c) == 3:  # Ctrl-C
                rclpy.shutdown(); return
            if ord(c) == 27: # ESC
                rclpy.shutdown(); return
            self._handle_key(c)

        # publish
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)

    def _kbhit(self):
        return select.select([sys.stdin], [], [], 0)[0] != []

    def _clamp_angles(self):
        # keep angles within [-pi, pi] for readability
        for name in ("roll", "pitch", "yaw"):
            v = getattr(self, name)
            if v > math.pi or v < -math.pi:
                v = (v + math.pi) % (2*math.pi) - math.pi
                setattr(self, name, v)

    def _print_pose(self):
        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        print(f"\nframe_id: {self.frame_id}")
        print(f"pos  [m]:  x={self.x:+.3f}  y={self.y:+.3f}  z={self.z:+.3f}")
        print(f"rpy [deg]: roll={math.degrees(self.roll):+6.1f}  "
              f"pitch={math.degrees(self.pitch):+6.1f}  yaw={math.degrees(self.yaw):+6.1f}")
        print(f"quat     : x={qx:+.4f} y={qy:+.4f} z={qz:+.4f} w={qw:+.4f}")

    def _handle_key(self, c):
        s = self.lin_step
        a = self.ang_step
        ch = c.lower()

        if ch == 'w': self.x += s
        elif ch == 's': self.x -= s
        elif ch == 'a': self.y += s
        elif ch == 'd': self.y -= s
        elif ch == 'r': self.z += s
        elif ch == 'f': self.z -= s

        elif ch == 'u': self.roll  += a
        elif ch == 'o': self.roll  -= a
        elif ch == 'i': self.pitch += a
        elif ch == 'k': self.pitch -= a
        elif ch == 'j': self.yaw   += a
        elif ch == 'l': self.yaw   -= a

        elif ch == 'g':  # zero orientation only
            self.roll = self.pitch = self.yaw = 0.0
        elif ch == ' ':  # zero everything
            self.x = self.y = self.z = 0.0
            self.roll = self.pitch = self.yaw = 0.0
        elif ch == 'p':
            self._print_pose()
            return
        elif ch in ['h', '?']:
            print(HELP, end="")
            return
        elif ch == 'q':
            rclpy.shutdown(); return
        else:
            return  # ignore unknown keys

        self._clamp_angles()
        # brief, inline status line
        sys.stdout.write(f"\rXYZ=({self.x:+.3f},{self.y:+.3f},{self.z:+.3f})  "
                         f"RPY(deg)=({math.degrees(self.roll):+5.1f},"
                         f"{math.degrees(self.pitch):+5.1f},{math.degrees(self.yaw):+5.1f})     ")
        sys.stdout.flush()

def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop for PoseStamped")
    parser.add_argument("--topic", default="/desired_pose",
                        help="PoseStamped topic to publish to")
    parser.add_argument("--frame-id", default="base_link",
                        help="Pose frame_id")
    parser.add_argument("--rate", type=float, default=30.0,
                        help="Publish rate (Hz)")
    parser.add_argument("--lin-step", type=float, default=0.01,
                        help="Linear step (m) per key press")
    parser.add_argument("--ang-step-deg", type=float, default=2.0,
                        help="Angular step (degrees) per key press")
    args = parser.parse_args()

    rclpy.init()
    node = PoseTeleop(args.topic, args.frame_id, args.rate, args.lin_step, args.ang_step_deg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown() is called inside on quit paths; call again safely:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
