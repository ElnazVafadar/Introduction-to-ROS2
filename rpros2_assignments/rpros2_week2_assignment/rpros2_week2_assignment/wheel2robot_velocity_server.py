#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

# Import custom service
from rpros2_interfaces.srv import Wheel2RobotVelocity


class Wheel2RobotVelocityServer(Node):
    def __init__(self):
        super().__init__('wheel2robot_velocity_server')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_distance', 0.5)
        self.declare_parameter('wheel_velocity_unit', 'rpm')  # rpm or m/s

        # Create the service
        self.srv = self.create_service(
            Wheel2RobotVelocity,
            'compute_wheel_velocities',
            self.compute_callback
        )

        self.get_logger().info("Wheel2RobotVelocity Server is running...")

    def compute_callback(self, request, response):

        # Get parameters
        R = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_distance').value
        unit = self.get_parameter('wheel_velocity_unit').value

        v = request.v
        w = request.w

        # Differential drive equations
        v_r = (2 * v + w * L) / 2
        v_l = (2 * v - w * L) / 2

        # Convert to wheel angular velocity
        # wheel angular velocity = linear_velocity / radius
        w_r = v_r / R
        w_l = v_l / R

        # Convert to rpm if required
        if unit == 'rpm':
            w_r = (w_r * 60) / (2 * math.pi)
            w_l = (w_l * 60) / (2 * math.pi)

        response.v_r = w_r
        response.v_l = w_l

        self.get_logger().info(
            f"Request -> v={v} m/s, w={w} rad/s | "
            f"Response -> v_l={w_l:.3f}, v_r={w_r:.3f} ({unit})"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Wheel2RobotVelocityServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

