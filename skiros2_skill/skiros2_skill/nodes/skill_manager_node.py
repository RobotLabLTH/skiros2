from skiros2_skill.ros.skill_manager import SkillManagerNode
import rclpy
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = SkillManagerNode()
    node.disallow_spinning()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
