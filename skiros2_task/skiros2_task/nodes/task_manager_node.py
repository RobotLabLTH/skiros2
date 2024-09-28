from skiros2_task.ros.task_manager import *
import rclpy

def main():
    rclpy.init()
    node = TaskManagerNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
