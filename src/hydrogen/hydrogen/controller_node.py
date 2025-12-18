#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/controller_output',
            10
        )

    def imu_callback(self, imu_msg: Imu):
        out = String()

        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w

        wx = imu_msg.angular_velocity.x
        wy = imu_msg.angular_velocity.y
        wz = imu_msg.angular_velocity.z

        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        out.data = (
            f"q=(x={qx:.6f}, y={qy:.6f}, z={qz:.6f}, w={qw:.6f}) "
            f"gyro=(x={wx:.6f}, y={wy:.6f}, z={wz:.6f}) "
            f"accel=(x={ax:.6f}, y={ay:.6f}, z={az:.6f})"
        )

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

