#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Int32

from cms50dplus_driver.cms50dplus import CMS50DPlus


class DriverNode(Node):

    def __init__(self) -> None:

        super().__init__("driver_node")

        self.serialport = self.declare_parameter("serialport", "/dev/ttyUSB0").get_parameter_value().string_value
        self.bpm_publisher = self.create_publisher(Int32, "bpm", 10)
        self.spo2_publisher = self.create_publisher(Int32, "spo2", 10)
        self.wave_publisher = self.create_publisher(Int32, "wave", 10)

        self.driver = CMS50DPlus(self.serialport)

        self.get_logger().info("Started driver_node")


def spin_node(node: DriverNode):

    data = node.driver.loop()
    if data is not None:
        bpm, spo2, wave = data

        node.get_logger().info(f"bpm: {bpm}, spo2: {spo2}, wave: {wave}")

        node.bpm_publisher.publish(Int32(data=bpm))
        node.spo2_publisher.publish(Int32(data=spo2))
        node.wave_publisher.publish(Int32(data=wave))

    # spin is not necessary for only publishing
    # rclpy.spin_once(node)


def main(args=None):

    rclpy.init(args=args)

    node = DriverNode()

    try:
        while rclpy.ok():
            spin_node(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.driver.timer.stop()
        node.get_logger().info('Recording stopped.')

        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
