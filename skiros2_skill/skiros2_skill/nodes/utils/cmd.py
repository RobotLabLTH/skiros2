import skiros2_skill.ros.skill_layer_interface as sli
import sys
import rclpy


def main():
    if len(sys.argv) < 4:
        print("usage: cmd skill_mgr_name start|stop skill_name|execution_id wait")
    else:
        mgr = sys.argv[1]
        action = sys.argv[2]
        skill = sys.argv[3]
        wait = False
        rclpy.init()
        node = rclpy.make_node("cmd")
        if len(sys.argv) > 4:
            wait = True
        si = sli.SkillLayerInterface(node)
        agent = si.getAgent(mgr)
        if action == "start":
            eid = agent.execute([agent.getSkill(skill)], "cmd_line_tool")
            if wait:
                agent.waitResult(eid)  # Bugged
            else:
                print(eid)
        if action == "stop":
            agent.preempt(int(skill), "cmd_line_tool")


if __name__ == '__main__':
    main()
