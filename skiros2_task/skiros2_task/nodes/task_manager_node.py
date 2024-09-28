from skiros2_task.ros.task_manager import *
import rclpy
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()
    node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
