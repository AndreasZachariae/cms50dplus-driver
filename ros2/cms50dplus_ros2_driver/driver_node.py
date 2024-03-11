#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class DriverNode(Node):

    def __init__(self) -> None:

        super().__init__("driver_node")

        self.serialport = self.declare_parameter("serialport", "/dev/ttyUSB0").get_parameter_value().string_value

        self.get_logger().info("Started driver_node")


def main(args=None):

    rclpy.init(args=args)

    node = DriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
