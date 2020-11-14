import rospy

from skiros2_common.tools.decorators import PrettyObject

import actionlib
import skiros2_msgs.msg as msgs
import skiros2_common.tools.logger as log


class TaskManagerInterface(PrettyObject):
    def __init__(self):
        """
        @brief      Interface for the task planning action server
        """
        self._assign_task_client = actionlib.SimpleActionClient('/tm/task_plan', msgs.AssignTaskAction)
        self._assign_task_client.wait_for_server()

    def plan(self, goals, done_cb=None, feedback_cb=None):
        """
        @brief      Send goals to task planner action server

        @param      goals        list(string) List of goals in PDDL format e.g.
                                 ["(skiros:contain skiros:LargeBox-80
                                 skiros:Starter-145)"]
        @param      done_cb      (function) Callback on done
        @param      feedback_cb  (function) Callback on feedback

        @return     (bool) True if action server is found, False otherwise
        """
        req = msgs.AssignTaskGoal()
        req.goals = goals
        if not self._assign_task_client.wait_for_server(rospy.Duration(0.1)):
            log.error("[TaskManagerInterface]", "Action server is not available.")
            return False
        self._assign_task_client.send_goal(req, done_cb=done_cb, feedback_cb=feedback_cb)
        return True

    def preempt(self):
        """
        @brief      Cancel goals for the task manager
        """
        self._assign_task_client.cancel_all_goals()

    def wait_for_result(self):
        """
        @brief      Wait for the task manager

        @return     { description_of_the_return_value }
        """
        self._assign_task_client.wait_for_result()
        return self._assign_task_client.get_result()
