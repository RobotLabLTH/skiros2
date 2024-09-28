from rclpy import action, task

from skiros2_common.tools.decorators import PrettyObject

import skiros2_msgs.msg as msgs
import skiros2_msgs.action as action_msgs
import skiros2_common.tools.logger as log


class TaskManagerInterface(PrettyObject):
    def __init__(self, node):
        """
        @brief      Interface for the task planning action server
        """
        self._assign_task_client = action.ActionClient(node, action_msgs.AssignTask, '/tm/task_plan')
        self._assign_task_client.wait_for_server()
        self._goal_handle = None

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
        req = action_msgs.AssignTask.Goal()
        req.goals = goals
        if not self._assign_task_client.wait_for_server(0.1):
            log.error("[TaskManagerInterface]", "Action server is not available.")
            return False
        self._goal_future = self._assign_task_client.send_goal_async(req, feedback_callback = feedback_cb)

        def _result_cb(future: task.Future):
            goal_handle = future.result()
            def _done_cb(future):
                status = future.result().status # type: GoalStatus
                result = future.result().result
                done_cb(status, result)

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_done_cb)
        
        self._goal_future.add_done_callback(_result_cb)
        return True

    def preempt(self):
        """
        @brief      Cancel goals for the task manager
        """
        log.info("Cancel requested")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            log.info("Cancel requested completed.")
        else:
            log.info("Goal has not been accepted or rejected, cannot cancel.")

    def wait_for_result(self):
        """
        @brief      Wait for the task manager

        @return     { description_of_the_return_value }
        """
        self._assign_task_client.wait_for_result()
        return self._assign_task_client.get_result()
