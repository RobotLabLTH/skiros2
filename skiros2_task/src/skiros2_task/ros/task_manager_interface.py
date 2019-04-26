import rospy

from skiros2_common.tools.decorators import PrettyObject

import actionlib
import skiros2_msgs.msg as msgs
import skiros2_common.tools.logger as log


class TaskManagerInterface(PrettyObject):
    def __init__(self):
        self._assign_task_client = actionlib.SimpleActionClient('/tm/task_plan', msgs.AssignTaskAction)
        self._assign_task_client.wait_for_server()

    def plan(self, goals, done_cb=None, feedback_cb=None):
        """
        @brief Send goals to task manager. If the task is accepted returns True, False otherwise
        """
        req = msgs.AssignTaskGoal()
        req.goals = goals
        if not self._assign_task_client.wait_for_server(rospy.Duration(0.1)):
            log.error("[TaskManagerInterface]", "Action server is not available.")
            return False
        self._assign_task_client.send_goal(req, done_cb=done_cb, feedback_cb=feedback_cb)
        return True

    def preempt(self):
        self._assign_task_client.cancel_all_goals()

    def wait_for_result(self):
        self._assign_task_client.wait_for_result()
        return self._assign_task_client.get_result()
