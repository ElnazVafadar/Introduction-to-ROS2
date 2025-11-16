#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from rpros2_interfaces.srv import Wheel2RobotVelocity


class Wheel2RobotVelocityClient(Node):
    def __init__(self):
        super().__init__('wheel2robot_velocity_client')

        # Create client
        self.client = self.create_client(Wheel2RobotVelocity, 'compute_wheel_velocities')

        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.req = Wheel2RobotVelocity.Request()

    def send_request(self, v, w):
        self.req.v = float(v)
        self.req.w = float(w)

        return self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    client = Wheel2RobotVelocityClient()

    if len(sys.argv) < 3:
        print("Usage: ros2 run rpros2_week2_assignment wheel2robot_vel_client <v> <w>")
        return

    v = float(sys.argv[1])
    w = float(sys.argv[2])

    future = client.send_request(v, w)

    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        print(f"\n💡 Wheel Velocities:")
        print(f"Left Wheel  = {future.result().v_l}")
        print(f"Right Wheel = {future.result().v_r}\n")
    else:
        print("Service call failed!")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

