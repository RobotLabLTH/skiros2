#!/usr/bin/env python

from skiros2_world_model.ros.ontology_server import OntologyServer
import rclpy


def main():
    rclpy.init()
    node = OntologyServer()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
