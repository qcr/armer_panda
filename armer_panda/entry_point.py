#!/usr/bin/env python3
"""The Armer Panda ROS2 Node Class

This class handles the ROS2 function of the ARMer Panda class
"""

from __future__ import annotations

__author__ = ['Dasun Gunasinghe']
__version__ = "0.1.0"

import os
import rclpy
import ament_index_python
import numpy as np
from armer.armer import Armer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

__path__ = ament_index_python.packages.get_package_share_directory('armer_panda')

def main(args=None):
    """
    Initialises and executes the node
    """
    rclpy.init(args=args)

    try:
      armer = ArmerNode()
      executor = MultiThreadedExecutor(num_threads=4)
      executor.add_node(armer)

      try:
          executor.spin()
      finally:
          executor.shutdown()
          armer.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
  main()