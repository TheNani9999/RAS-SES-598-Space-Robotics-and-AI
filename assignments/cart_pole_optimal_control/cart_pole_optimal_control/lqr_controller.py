#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')

        # System parameters
        self.M = 1.1  # Mass of cart (kg)
        self.m = 0.95  # Mass of pole (kg)
        self.L = 1.1  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)

        # State-space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])

        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])

        # LQR cost matrices (Tuned)
        self.Q = np.diag([100.0, 50.0, 100.0, 10.0])  # Higher weight on position and angle
        self.R = np.array([[0.01]])  # Lower cost on force for aggressive control

        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')

        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False

        # Desired cart position (oscillation)
        self.desired_cart_position = 0.0
        self.desired_cart_velocity = 0.0

        # Create publishers
        self.cart_cmd_pub = self.create_publisher(Float64, '/model/cart_pole/joint/cart_to_base/cmd_force', 10)
        self.state_pub = self.create_publisher(Float64MultiArray, '/cart_pole/state', 10)

        # Create subscriber for joint states
        self.joint_state_sub = self.create_subscription(JointState, '/world/empty/model/cart_pole/joint_state', self.joint_state_callback, 10)

        # Control loop timer (update every 5ms)
        self.timer = self.create_timer(0.005, self.control_loop)

        self.get_logger().info('Cart-Pole LQR Controller initialized')

    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')

            new_x = np.array([
                [msg.position[cart_idx]],
                [msg.velocity[cart_idx]],
                [msg.position[pole_idx]],
                [msg.velocity[pole_idx]]
            ])

            # Apply exponential moving average filter for noise reduction
            if self.state_initialized:
                self.x = 0.85 * self.x + 0.15 * new_x
            else:
                self.x = new_x
                self.state_initialized = True

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}')

    def control_loop(self):
        """Compute and apply LQR control."""
        if not self.state_initialized:
            return

        # Reference state
        x_ref = np.array([
            [self.desired_cart_position],
            [self.desired_cart_velocity],
            [0.0],  # Pole should remain upright (0 radians)
            [0.0]
        ])

        # Compute control input
        x_error = self.x - x_ref
        u = -self.K @ x_error

        # Clip force to physical actuator limits
        force = np.clip(float(u[0]), -15.0, 15.0)

        # Publish control force
        msg = Float64()
        msg.data = force
        self.cart_cmd_pub.publish(msg)

        # Publish state data for external plotting
        state_msg = Float64MultiArray()
        state_msg.data = [self.x[0, 0], self.x[1, 0], self.x[2, 0], self.x[3, 0], force]
        self.state_pub.publish(state_msg)

        # Introduce a small sinusoidal oscillation in the reference position
        self.desired_cart_position = 0.5 * np.sin(self.get_clock().now().nanoseconds * 1e-9 * 0.5)

def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
