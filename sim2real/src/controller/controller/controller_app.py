#!/usr/bin/env python

import sys
import rclpy
from rclpy.executors import SingleThreadedExecutor
from controller.controller_node import ControllerNode

def main():
    # Initialize ROS 2 context and node
    rclpy.init(args=sys.argv)
    controller_node = ControllerNode()

    executor = SingleThreadedExecutor()
    executor.add_node(controller_node)

    # Run the node
    try:
        executor.spin()
    except KeyboardInterrupt:
        controller_node.cleanup()
        controller_node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()
