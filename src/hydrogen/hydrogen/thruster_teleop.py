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

       
        self.pubs = {
            'front_propeller': self.create_publisher(Float64, '/hydrogen/front_propeller/cmd_thrust', 10),
            'right_propeller_1': self.create_publisher(Float64, '/hydrogen/right_propeller_1/cmd_thrust', 10),
            'right_propeller_2': self.create_publisher(Float64, '/hydrogen/right_propeller_2/cmd_thrust', 10),
            'left_propeller_1':  self.create_publisher(Float64, '/hydrogen/left_propeller_1/cmd_thrust', 10),
            'left_propeller_2':  self.create_publisher(Float64, '/hydrogen/left_propeller_2/cmd_thrust', 10),
        }

        self.values = {k: 0.0 for k in self.pubs.keys()}
        self.step = 2.0
        self.scale = 1.0
        self.max_thrust = 100.0

        self.timer = self.create_timer(0.1, self.publish_all)

    def publish_all(self):
        """ Continuously publish all thruster commands """
        for k, v in self.values.items():
            msg = Float64()
            msg.data = float(v)
            self.pubs[k].publish(msg)

    def stop_all(self):
        for k in self.values:
            self.values[k] = 0.0
        self.publish_all()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("thruster teleop chal raha hai")
    print("Use Arrow Keys + W/S + I/K + J/L")
    print("Press SPACE to stop, X to exit\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("Exiting teleop.")
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
                        node.values['front_propeller'] += node.step
                        node.values['left_propeller_2'] += node.step
                        node.values['right_propeller_2'] += node.step

                    elif seq == '[B':   # DOWN ARROW (descend)
                        node.values['front_propeller'] -= node.step
                        node.values['left_propeller_2'] -= node.step
                        node.values['right_propeller_2'] -= node.step

                    elif seq == '[C':   # RIGHT ARROW → yaw right
                        node.values['left_propeller_1'] -= node.step
                        node.values['right_propeller_1'] += node.step

                    elif seq == '[D':   # LEFT ARROW → yaw left
                        node.values['left_propeller_1'] += node.step
                        node.values['right_propeller_1'] -= node.step

                else:
                    ch = ch.lower()

                    # Forward / backward surge
                    if ch == 'w':
                        node.values['left_propeller_1'] += node.step
                        node.values['right_propeller_1'] += node.step
                    elif ch == 's':
                        node.values['left_propeller_1'] -= node.step
                        node.values['right_propeller_1'] -= node.step

                    # Pitch (I/K)
                    elif ch == 'i':  # nose down
                        node.values['left_propeller_2'] += node.step
                        node.values['right_propeller_2'] -= node.step
                    elif ch == 'k':  # nose up
                        node.values['left_propeller_2'] -= node.step
                        node.values['right_propeller_2'] += node.step

                    # Roll (J/L)
                    elif ch == 'j':
                        node.values['left_propeller_2'] -= node.step
                        node.values['right_propeller_2'] += node.step
                    elif ch == 'l':
                        node.values['left_propeller_2'] += node.step
                        node.values['right_propeller_2'] -= node.step

                    elif ch == ' ':
                        node.stop_all()

                    elif ch == 'x':
                        exit_clean()

                for k in node.values:
                    node.values[k] = clamp(node.values[k], -node.max_thrust, node.max_thrust)

                sys.stdout.write(
                    f"\rfront:{node.values['front_propeller']:.1f}  "
                    f"L1:{node.values['left_propeller_1']:.1f}  "
                    f"R1:{node.values['right_propeller_1']:.1f}  "
                    f"L2:{node.values['left_propeller_2']:.1f}  "
                    f"R2:{node.values['right_propeller_2']:.1f}   "
                )
                sys.stdout.flush()

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()


if __name__ == "__main__":
    main()

