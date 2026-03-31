#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, signal, math
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def quaternion_to_pitch(x, y, z, w):
    """Convert quaternion to pitch angle in radians (rotation about y-axis)."""
    sinp = 2.0 * (w * y - z * x)
    sinp = clamp(sinp, -1.0, 1.0)
    return math.asin(sinp)


class PID:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self._integral  += error * dt
        derivative       = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return clamp(output, self.output_min, self.output_max)

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_thrusters")

        # ================= Controller Subscriptions =================
        self.sub_front = self.create_subscription(
            Float64, 'new_thrust_front', self.cb_front, 10)
        self.sub_left = self.create_subscription(
            Float64, 'new_thrust_left', self.cb_left, 10)
        self.sub_right = self.create_subscription(
            Float64, 'new_thrust_right', self.cb_right, 10)

        # ================= IMU Subscription =================
        self.sub_imu = self.create_subscription(
            Imu, '/imu', self.cb_imu, 10)

        # ================= Thruster Publishers =================
        self.pubs = {
            'back_propeller':    self.create_publisher(Float64, '/hydrogen/back_propeller/cmd_thrust', 10),
            'right_propeller_1': self.create_publisher(Float64, '/hydrogen/right_propeller_1/cmd_thrust', 10),
            'right_propeller_2': self.create_publisher(Float64, '/hydrogen/right_propeller_2/cmd_thrust', 10),
            'left_propeller_1':  self.create_publisher(Float64, '/hydrogen/left_propeller_1/cmd_thrust', 10),
            'left_propeller_2':  self.create_publisher(Float64, '/hydrogen/left_propeller_2/cmd_thrust', 10),
        }

        # ================= Controller Values =================
        self.ctrl_values = {
            'back_propeller':    0.0,
            'left_propeller_2':  0.0,
            'right_propeller_2': 0.0,
        }

        # ================= Horizontal Motion Offsets (reset every cycle) =================
        self.manual_offsets = {k: 0.0 for k in self.pubs.keys()}

        # ================= Tuning Parameters =================
        self.step           = 50.0
        self.horizontal_max = 50.0
        self.vertical_max   = 100.0

        # ================= Depth Toggle State (PERSISTENT) =================
        self.vertical_thrust = 0.0
        self.vertical_step   = 5.0

        # ================= Back Thruster Base Ratio =================
        # Calculated from CoM distances:
        # d_front=144.18mm, d_back=353.98mm → ratio = (2 * 144.18) / 353.98 = 0.815
        # PID correction adds on top of this in real time
        self.back_thrust_ratio = 0.815

        # ================= IMU / Pitch State =================
        self.pitch_angle = 0.0          # radians, updated by IMU callback

        # ================= Pitch PID =================
        # Target: pitch = 0.0 (level)
        # Output: correction added to back_propeller thrust only during ascend/descend
        #
        # Tuning guide:
        #   kp — increase if correction is too slow to respond
        #   ki — increase if nose settles at a small offset and never fully corrects
        #   kd — increase if correction oscillates back and forth
        self.pitch_pid = PID(
            kp=20.0,
            ki=0.5,
            kd=5.0,
            output_min=-30.0,
            output_max=30.0
        )

        self._last_pid_time = self.get_clock().now()

        self.timer = self.create_timer(0.02, self.publish_all)

    # ================= Controller Callbacks =================
    def cb_front(self, msg):
        self.ctrl_values['back_propeller'] = msg.data

    def cb_left(self, msg):
        self.ctrl_values['left_propeller_2'] = msg.data

    def cb_right(self, msg):
        self.ctrl_values['right_propeller_2'] = msg.data

    # ================= IMU Callback =================
    def cb_imu(self, msg):
        o = msg.orientation
        self.pitch_angle = quaternion_to_pitch(o.x, o.y, o.z, o.w)

    # ================= Publishing =================
    def publish_all(self):

        # Compute PID correction only when actively ascending/descending
        pid_correction = 0.0
        if self.vertical_thrust != 0.0:
            now = self.get_clock().now()
            dt  = (now - self._last_pid_time).nanoseconds * 1e-9
            self._last_pid_time = now

            # error = target pitch (0 = level) minus current pitch
            # nose down → negative pitch → positive error → positive correction → more back thrust
            # nose up   → positive pitch → negative error → negative correction → less back thrust
            error          = 0.0 - self.pitch_angle
            pid_correction = self.pitch_pid.compute(error, dt)
        else:
            # Reset PID when not moving vertically to prevent integral windup
            self.pitch_pid.reset()
            self._last_pid_time = self.get_clock().now()

        for name, pub in self.pubs.items():
            ctrl = self.ctrl_values.get(name, 0.0)

            """
            OLD CODE — single ratio for all vertical thrusters, no back_thrust_ratio:
            if name in ('left_propeller_2', 'right_propeller_2'):
                blended = ctrl + self.manual_offsets[name] + self.vertical_thrust
            else:
                blended = ctrl + self.manual_offsets[name]
            """

            if name == 'back_propeller':
                # Base ratio keeps vehicle level in theory (from CoM moment calculation)
                # PID correction adjusts in real time based on actual pitch from IMU
                blended = ctrl + self.manual_offsets[name] \
                          + (self.vertical_thrust * self.back_thrust_ratio) \
                          + pid_correction

            elif name in ('left_propeller_2', 'right_propeller_2'):
                blended = ctrl + self.manual_offsets[name] + self.vertical_thrust

            else:
                blended = ctrl + self.manual_offsets[name]

            blended = clamp(blended, -self.vertical_max, self.vertical_max)

            msg = Float64()
            msg.data = float(blended)
            pub.publish(msg)

    def stop_all(self):
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.vertical_thrust = 0.0
        self.pitch_pid.reset()
        self.publish_all()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
===== Hydrogen AUV Teleop =====

Thrusters with controller + keyboard blending:
  back_propeller, left_propeller_2, right_propeller_2

Thrusters with keyboard only:
  left_propeller_1, right_propeller_1

Controls:
  A / D           : Ascend / Descend  (TOGGLE — persists after release)
  W / S           : Forward / Backward
  Arrow Left/Right: Yaw
  I / K           : Pitch up / down
  L / R           : Roll left / right
  SPACE           : Reset ALL (including depth)
  X               : Exit
================================
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():

            # Reset horizontal offsets every cycle
            # vertical_thrust is NOT reset — persists for depth holding
            for k in node.manual_offsets:
                node.manual_offsets[k] = 0.0

            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b':
                    seq = sys.stdin.read(2)

                    if seq == '[C':    # yaw right
                        node.manual_offsets['left_propeller_1']  =  node.step
                        node.manual_offsets['right_propeller_1'] = -node.step

                    elif seq == '[D':  # yaw left
                        node.manual_offsets['left_propeller_1']  = -node.step
                        node.manual_offsets['right_propeller_1'] =  node.step

                else:
                    ch = ch.lower()

                    if ch == 'w':    # forward
                        node.manual_offsets['left_propeller_1']  = -node.step
                        node.manual_offsets['right_propeller_1'] = -node.step

                    elif ch == 's':  # backward
                        node.manual_offsets['left_propeller_1']  =  node.step
                        node.manual_offsets['right_propeller_1'] =  node.step

                    elif ch == 'a':  # ascend
                        node.vertical_thrust = clamp(
                            node.vertical_thrust + node.vertical_step,
                            -node.vertical_max, node.vertical_max
                        )
                        print(f"  Depth: {node.vertical_thrust:+.1f}  |  back_base: {node.vertical_thrust * node.back_thrust_ratio:+.1f}  |  pitch: {math.degrees(node.pitch_angle):+.2f} deg")

                    elif ch == 'd':  # descend
                        node.vertical_thrust = clamp(
                            node.vertical_thrust - node.vertical_step,
                            -node.vertical_max, node.vertical_max
                        )
                        print(f"  Depth: {node.vertical_thrust:+.1f}  |  back_base: {node.vertical_thrust * node.back_thrust_ratio:+.1f}  |  pitch: {math.degrees(node.pitch_angle):+.2f} deg")

                    elif ch == 'i':  # pitch down
                        node.manual_offsets['left_propeller_2']  =  node.step
                        node.manual_offsets['right_propeller_2'] = -node.step

                    elif ch == 'k':  # pitch up
                        node.manual_offsets['left_propeller_2']  = -node.step
                        node.manual_offsets['right_propeller_2'] =  node.step

                    elif ch == 'l':  # roll left
                        node.manual_offsets['left_propeller_2']  = -node.step
                        node.manual_offsets['right_propeller_2'] =  node.step

                    elif ch == 'r':  # roll right
                        node.manual_offsets['left_propeller_2']  =  node.step
                        node.manual_offsets['right_propeller_2'] = -node.step

                    elif ch == ' ':  # reset everything
                        node.stop_all()
                        print("  All offsets cleared. Depth: 0.0")

                    elif ch == 'x':
                        exit_clean()

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()


if __name__ == "__main__":
    main()
