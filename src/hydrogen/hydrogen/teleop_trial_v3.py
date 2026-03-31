#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, signal
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


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

        # ================= Thruster Publishers =================
        self.pubs = {
            'back_propeller':    self.create_publisher(Float64, '/hydrogen/back_propeller/cmd_thrust', 10),
            'right_propeller_1': self.create_publisher(Float64, '/hydrogen/right_propeller_1/cmd_thrust', 10),
            'right_propeller_2': self.create_publisher(Float64, '/hydrogen/right_propeller_2/cmd_thrust', 10),
            'left_propeller_1':  self.create_publisher(Float64, '/hydrogen/left_propeller_1/cmd_thrust', 10),
            'left_propeller_2':  self.create_publisher(Float64, '/hydrogen/left_propeller_2/cmd_thrust', 10),
        }

        # ================= Controller Values =================
        # Only 3 thrusters receive autonomous controller input
        self.ctrl_values = {
            'back_propeller':    0.0,
            'left_propeller_2':  0.0,
            'right_propeller_2': 0.0,
        }

        # ================= Horizontal Motion Offsets (reset every cycle) =================
        self.manual_offsets = {k: 0.0 for k in self.pubs.keys()}

        # ================= Tuning Parameters =================
        self.step           = 50.0   # horizontal thrust per keypress
        self.horizontal_max = 50.0   # clamp for horizontal thrusters
        self.vertical_max   = 100.0  # clamp for vertical thrusters (higher to fight buoyancy)

        # ================= Depth Toggle State (PERSISTENT) =================
        # A/D increment/decrement this value — it persists after key release
        self.vertical_thrust = 0.0
        self.vertical_step   = 5.0   # how much each A/D press changes depth

        # ================= Back Thruster Ratio =================
        # back_propeller is further from CoM than front thrusters
        # ratio = (2 * d_front) / d_back = (2 * 144.18) / 353.98 = 0.815
        # Tune this if vehicle still tips during ascend/descend
        self.back_thrust_ratio = 1.2

        self.timer = self.create_timer(0.02, self.publish_all)

    # ================= Controller Callbacks =================
    def cb_front(self, msg):
        self.ctrl_values['back_propeller'] = msg.data

    def cb_left(self, msg):
        self.ctrl_values['left_propeller_2'] = msg.data

    def cb_right(self, msg):
        self.ctrl_values['right_propeller_2'] = msg.data

    # ================= Publishing =================
    def publish_all(self):
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
                # Rear vertical thruster — scaled by ratio to keep vehicle level
                # d_front=144.18mm, d_back=353.98mm → ratio=0.815
                blended = ctrl + self.manual_offsets[name] + (self.vertical_thrust * self.back_thrust_ratio)

            elif name in ('left_propeller_2', 'right_propeller_2'):
                # Front vertical thrusters — full vertical_thrust value
                blended = ctrl + self.manual_offsets[name] + self.vertical_thrust

            else:
                # Horizontal thrusters — no vertical component
                blended = ctrl + self.manual_offsets[name]

            blended = clamp(blended, -self.vertical_max, self.vertical_max)

            msg = Float64()
            msg.data = float(blended)
            pub.publish(msg)

    def stop_all(self):
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.vertical_thrust = 0.0
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

            # Reset horizontal offsets every cycle — keyboard must be held for continuous motion
            # vertical_thrust is NOT reset — it persists for depth holding
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

                    elif ch == 'a':  # ascend — increment persistent depth
                        node.vertical_thrust = clamp(
                            node.vertical_thrust + node.vertical_step,
                            -node.vertical_max, node.vertical_max
                        )
                        print(f"  Depth: {node.vertical_thrust:+.1f}  |  back: {node.vertical_thrust * node.back_thrust_ratio:+.1f}")

                    elif ch == 'd':  # descend — decrement persistent depth
                        node.vertical_thrust = clamp(
                            node.vertical_thrust - node.vertical_step,
                            -node.vertical_max, node.vertical_max
                        )
                        print(f"  Depth: {node.vertical_thrust:+.1f}  |  back: {node.vertical_thrust * node.back_thrust_ratio:+.1f}")

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
