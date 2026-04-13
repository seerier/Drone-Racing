#!/usr/bin/env python

import sys
import rclpy
from rclpy.executors import SingleThreadedExecutor
from crazyradio_driver.crazyradio_driver_node import CrazyradioDriverNode

def main():
    # Initialize ROS 2 context and node
    rclpy.init(args=sys.argv)
    crazyradio_driver_node = CrazyradioDriverNode()

    executor = SingleThreadedExecutor()
    executor.add_node(crazyradio_driver_node)

    # Run the node
    try:
        executor.spin()
    except KeyboardInterrupt:
        crazyradio_driver_node.cleanup()
        crazyradio_driver_node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()
