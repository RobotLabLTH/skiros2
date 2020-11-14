#!/usr/bin/env python

from skiros2_world_model.ros.world_model_server import WorldModelServer
import rclpy


def main():
    rclpy.init()
    node = WorldModelServer()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
