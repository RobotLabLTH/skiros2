from skiros2_skill.ros.skill_manager import SkillManagerNode
import rclpy


def main():
    rclpy.init()
    node = SkillManagerNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
