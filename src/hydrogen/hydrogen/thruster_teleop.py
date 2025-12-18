#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, time, signal
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_thrusters")

        # Subscribe to controller outputs
        self.sub_front = self.create_subscription(
            Float64, '/hydrogen/front_propeller/cmd_thrust', 
            self.cb_front, 10
        )
        self.sub_left_1 = self.create_subscription(
            Float64, '/hydrogen/left_propeller_1/cmd_thrust',
            self.cb_left_1, 10
        )
        self.sub_right_1 = self.create_subscription(
            Float64, '/hydrogen/right_propeller_1/cmd_thrust',
            self.cb_right_1, 10
        )
        self.sub_left_2 = self.create_subscription(
            Float64, '/hydrogen/left_propeller_2/cmd_thrust',
            self.cb_left_2, 10
        )
        self.sub_right_2 = self.create_subscription(
            Float64, '/hydrogen/right_propeller_2/cmd_thrust',
            self.cb_right_2, 10
        )

        # Publishers (will override controller if teleop is active)
        self.pubs = {
            'front_propeller': self.create_publisher(Float64, '/hydrogen/front_propeller/cmd_thrust', 10),
            'right_propeller_1': self.create_publisher(Float64, '/hydrogen/right_propeller_1/cmd_thrust', 10),
            'right_propeller_2': self.create_publisher(Float64, '/hydrogen/right_propeller_2/cmd_thrust', 10),
            'left_propeller_1':  self.create_publisher(Float64, '/hydrogen/left_propeller_1/cmd_thrust', 10),
            'left_propeller_2':  self.create_publisher(Float64, '/hydrogen/left_propeller_2/cmd_thrust', 10),
        }

        # Controller-provided values (read from subscriptions)
        self.ctrl_values = {
            'front_propeller': 0.0,
            'right_propeller_1': 0.0,
            'right_propeller_2': 0.0,
            'left_propeller_1': 0.0,
            'left_propeller_2': 0.0,
        }

        # Manual offsets (added by teleop)
        self.manual_offsets = {k: 0.0 for k in self.pubs.keys()}
        
        self.step = 2.0
        self.scale = 1.0
        self.max_thrust = 100.0

        # Timer to publish blended values
        self.timer = self.create_timer(0.1, self.publish_all)

    # ===== Callbacks to read controller values =====
    def cb_front(self, msg):
        self.ctrl_values['front_propeller'] = msg.data

    def cb_left_1(self, msg):
        self.ctrl_values['left_propeller_1'] = msg.data

    def cb_right_1(self, msg):
        self.ctrl_values['right_propeller_1'] = msg.data

    def cb_left_2(self, msg):
        self.ctrl_values['left_propeller_2'] = msg.data

    def cb_right_2(self, msg):
        self.ctrl_values['right_propeller_2'] = msg.data

    def publish_all(self):
        """ Publish blended values: controller + manual offsets """
        for k in self.pubs.keys():
            blended = self.ctrl_values[k] + self.manual_offsets[k]
            blended = clamp(blended, -self.max_thrust, self.max_thrust)
            
            msg = Float64()
            msg.data = float(blended)
            self.pubs[k].publish(msg)

    def stop_all(self):
        """ Reset manual offsets to zero (keep controller running) """
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.publish_all()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("Teleop + Controller Blending Active")
    print("Controller outputs are automatically read and blended with manual inputs")
    print("Use Arrow Keys + W/S + I/K + J/L for manual offsets")
    print("Press SPACE to clear manual inputs (keep controller), X to exit\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nExiting teleop.")
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():

            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b':
                    seq = sys.stdin.read(2)

                    if seq == '[A':     # UP ARROW (ascend)
                        node.manual_offsets['front_propeller'] += node.step
                        node.manual_offsets['left_propeller_2'] += node.step
                        node.manual_offsets['right_propeller_2'] += node.step

                    elif seq == '[B':   # DOWN ARROW (descend)
                        node.manual_offsets['front_propeller'] -= node.step
                        node.manual_offsets['left_propeller_2'] -= node.step
                        node.manual_offsets['right_propeller_2'] -= node.step

                    elif seq == '[C':   # RIGHT ARROW → yaw right
                        node.manual_offsets['left_propeller_1'] -= node.step
                        node.manual_offsets['right_propeller_1'] += node.step

                    elif seq == '[D':   # LEFT ARROW → yaw left
                        node.manual_offsets['left_propeller_1'] += node.step
                        node.manual_offsets['right_propeller_1'] -= node.step

                else:
                    ch = ch.lower()

                    # Forward / backward surge
                    if ch == 'w':
                        node.manual_offsets['left_propeller_1'] += node.step
                        node.manual_offsets['right_propeller_1'] += node.step
                    elif ch == 's':
                        node.manual_offsets['left_propeller_1'] -= node.step
                        node.manual_offsets['right_propeller_1'] -= node.step

                    # Pitch (I/K)
                    elif ch == 'i':  # nose down
                        node.manual_offsets['left_propeller_2'] += node.step
                        node.manual_offsets['right_propeller_2'] -= node.step
                    elif ch == 'k':  # nose up
                        node.manual_offsets['left_propeller_2'] -= node.step
                        node.manual_offsets['right_propeller_2'] += node.step

                    # Roll (J/L)
                    elif ch == 'j':
                        node.manual_offsets['left_propeller_2'] -= node.step
                        node.manual_offsets['right_propeller_2'] += node.step
                    elif ch == 'l':
                        node.manual_offsets['left_propeller_2'] += node.step
                        node.manual_offsets['right_propeller_2'] -= node.step

                    elif ch == ' ':
                        node.stop_all()

                    elif ch == 'x':
                        exit_clean()

                # Clamp all manual offsets
                for k in node.manual_offsets:
                    node.manual_offsets[k] = clamp(node.manual_offsets[k], -node.max_thrust, node.max_thrust)

                # Display: controller value + manual offset = final output
                sys.stdout.write(
                    f"\rFront: {node.ctrl_values['front_propeller']:.1f}+{node.manual_offsets['front_propeller']:.1f}={node.ctrl_values['front_propeller']+node.manual_offsets['front_propeller']:.1f}  "
                    f"L1: {node.ctrl_values['left_propeller_1']:.1f}+{node.manual_offsets['left_propeller_1']:.1f}={node.ctrl_values['left_propeller_1']+node.manual_offsets['left_propeller_1']:.1f}  "
                    f"R1: {node.ctrl_values['right_propeller_1']:.1f}+{node.manual_offsets['right_propeller_1']:.1f}={node.ctrl_values['right_propeller_1']+node.manual_offsets['right_propeller_1']:.1f}  "
                    f"L2: {node.ctrl_values['left_propeller_2']:.1f}+{node.manual_offsets['left_propeller_2']:.1f}={node.ctrl_values['left_propeller_2']+node.manual_offsets['left_propeller_2']:.1f}  "
                    f"R2: {node.ctrl_values['right_propeller_2']:.1f}+{node.manual_offsets['right_propeller_2']:.1f}={node.ctrl_values['right_propeller_2']+node.manual_offsets['right_propeller_2']:.1f}   "
                )
                sys.stdout.flush()

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()


if __name__ == "__main__":
    main()
