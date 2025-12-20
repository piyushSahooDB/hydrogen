#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.g = 9.8
        self.m = 4.8
        self.Fb = 6.0 * self.g

        self.alpha = 0.98 
        self.dt = 0.01      

        self.K = np.array([
            [ 1.0000,  0.6161,  0.0000,  0.0000, -0.0000,  0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000],
            [ 0.0000,  0.0000,  1.0000,  0.6600,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000],
            [-0.0000, -0.0000,  0.0000,  0.0000,  1.0000,  0.8357,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000],
            [-0.0000,  0.0000, -0.0000,  0.0000,  0.0000,  0.0000,  0.7071,  1.7997, -0.0000, -0.0000, -0.0000, -0.0000],
            [ 0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.7071,  3.7997,  0.0000,  0.0000],
            [ 0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000, -0.0000, -0.0000,  1.0954,  2.4053]
        ])

        self.u_trim = np.array([
            0.0, 0.0, 0.0, 0.0, 0.0, self.m*self.g - self.Fb
        ])

        x1 = 0.37 / 2
        y1 = 0.24
        y2 = 0.08

        FtoX = np.array([
            [ y2,  y1,  y2],
            [-x1,  0.0, x1],
            [ 1.0, 1.0, 1.0]
        ])
        self.XtoF = np.linalg.inv(FtoX)

        self.max_thrust = 40.0  # N

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.x_hat = np.zeros(12)

        self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        self.thrusters = [
            self.create_publisher(Float64, 'new_thrust_front', 10),
            self.create_publisher(Float64, 'new_thrust_left', 10),
            self.create_publisher(Float64, 'new_thrust_right', 10),
        ]

    def imu_cb(self, msg: Imu):

        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z


        phi_acc = np.arctan2(ay, az)
        theta_acc = np.arctan2(-ax, np.sqrt(ay**2 + az**2))


        self.phi = self.alpha*(self.phi + p*self.dt) + (1-self.alpha)*phi_acc
        self.theta = self.alpha*(self.theta + q*self.dt) + (1-self.alpha)*theta_acc
        self.psi += r*self.dt

        self.x_hat = np.array([
            self.phi, p,
            self.theta, q,
            self.psi, r,
            0.0, 0.0,
            0.0, 0.0,  
            0.0, 0.0   
        ])

        tau = self.u_trim + self.K @ self.x_hat


        tau_phi = tau[0]
        tau_theta = tau[1]
        Fz = tau[5]

        generalized = np.array([tau_phi, tau_theta, Fz])

        thrusts = self.XtoF @ generalized

        for i in range(3):
            msg_out = Float64()
            msg_out.data = float(thrusts[i])
            self.thrusters[i].publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
