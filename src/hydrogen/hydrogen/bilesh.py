#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import serial  # Added for ESP32 communication
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # ================= Serial Setup =================
        # Adjust '/dev/ttyUSB0' to your actual ESP32 port
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.01)
            self.get_logger().info("Serial connected to ESP32")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")

        # ================= Physical / tuning parameters =================
        self.dt = 0.005  # 200 Hz
        self.alpha = 0.98  # complementary filter

        # Cascaded control gains
        self.Kp = np.diag([5.0, 5.0])   # outer PI proportional
        self.Ki = np.diag([1.0, 1.0])   # outer PI integral

        # Inner LQR gain (angular velocity -> thrusters directly)
        self.K_lqr = np.array([
            [10.0, 0.0],  # T1 response to p, q
            [0.0, 10.0],  # T2 response to p, q
            [5.0, 5.0]    # T3 response to p, q
        ])

        # Trim forces for hover
        self.F_trim = np.array([0.0, 0.0, 0.0]) # Set based on buoyancy
        self.max_thrust_offset = 400.0 # PWM units relative to 1500

        # Orientation / state
        self.phi = 0.0; self.theta = 0.0; self.psi = 0.0
        self.int_phi = 0.0; self.int_theta = 0.0
        self.p = 0.0; self.q = 0.0

        # Desired orientation
        self.phi_des = 0.0
        self.theta_des = 0.0

        # ROS subscriptions & publishers
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.thruster_pubs = [
            self.create_publisher(Float64, 'new_thrust_front', 10),
            self.create_publisher(Float64, 'new_thrust_left', 10),
            self.create_publisher(Float64, 'new_thrust_right', 10),
        ]

        # Main Loop Timer
        self.create_timer(self.dt, self.control_loop)

    def imu_cb(self, msg: Imu):
        """Optional: Use this if you have a separate ROS IMU source"""
        pass

    def read_esp32_serial(self):
        """Reads IMU data sent from the ESP32 over Serial"""
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("IMU"):
                    parts = line.split(',')
                    # Format: IMU, roll, pitch, heading, p, q, r
                    self.phi = float(parts[1])
                    self.theta = float(parts[2])
                    self.p = float(parts[4])
                    self.q = float(parts[5])
            except:
                pass

    def control_loop(self):
        # 1. Update state from Serial
        self.read_esp32_serial()

        # 2. Outer PI loop (orientation -> reference angular velocities)
        e_phi = self.phi_des - self.phi
        e_theta = self.theta_des - self.theta

        self.int_phi += e_phi * self.dt
        self.int_theta += e_theta * self.dt

        # Anti-windup
        self.int_phi = np.clip(self.int_phi, -10.0, 10.0)
        self.int_theta = np.clip(self.int_theta, -10.0, 10.0)

        omega_ref = np.array([
            self.Kp[0,0]*e_phi + self.Ki[0,0]*self.int_phi,
            self.Kp[1,1]*e_theta + self.Ki[1,1]*self.int_theta
        ])

        # 3. Inner LQR loop (angular velocity error -> thrust)
        omega = np.array([self.p, self.q])
        omega_error = omega - omega_ref

        # thrusts = F_trim - K*(omega - omega_ref)
        thrusts = self.F_trim - self.K_lqr @ omega_error
        
        # Clip to safe PWM offsets
        thrusts = np.clip(thrusts, -self.max_thrust_offset, self.max_thrust_offset)

        # 4. Publish to ROS (Optional)
        for i, val in enumerate(thrusts):
            msg = Float64()
            msg.data = float(val)
            self.thruster_pubs[i].publish(msg)

        # 5. Send to ESP32 over Serial
        # Format: "THR,t1,t2,t3\n"
        cmd = f"THR,{thrusts[0]:.2f},{thrusts[1]:.2f},{thrusts[2]:.2f}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
