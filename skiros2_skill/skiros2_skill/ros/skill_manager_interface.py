import skiros2_msgs.msg as msgs
import skiros2_msgs.srv as srvs
from std_msgs.msg import Empty, Bool
import skiros2_common.ros.utils as utils
from skiros2_skill.ros.utils import SkillHolder
import skiros2_common.tools.logger as log
from multiprocessing import Lock, Event
import rclpy
from rclpy import subscription


class SkillManagerInterface:
    def __init__(self, node, manager_name, author_name, allow_spinning=True):
        self._skill_mgr_name = manager_name
        self._node = node
        self._author = author_name
        self._allow_spinning = allow_spinning
        self._active_tasks = set()
        self._module_list = dict()
        self._skill_list = None
        self._msg_lock = Lock()
        self._msg_rec = Event()
        self._skill_exe_client = self._node.create_client(srvs.SkillCommand, '/{}/command'.format(self._skill_mgr_name))
        self._get_skills = self._node.create_client(srvs.ResourceGetDescriptions, '/{}/get_skills'.format(self._skill_mgr_name))
        self._monitor_sub = self._node.create_subscription(msgs.TreeProgress, '/{}/monitor'.format(self._skill_mgr_name), self._progress_cb, 10)
        # TODO: find an equivalent of ROSTopicHz on ROS2
        # self._tick_rate = rostopic.ROSTopicHz(50)
        # self._tick_rate_sub = self._node.create_subscription(Empty, self._skill_mgr_name + '/tick_rate', self._tick_rate.callback_hz)
        self._set_debug = self._node.create_publisher(Bool, self._skill_mgr_name + "/set_debug", 1)
        self._monitor_cb = None
        for s in [self._skill_exe_client, self._get_skills]:
            while not s.wait_for_service(timeout_sec=1.0):
                log.warn("[{}]".format(self.__class__.__name__), "Service {} not available, waiting again ...".format(s.srv_name))

    @property
    def name(self):
        return self._skill_mgr_name

    @property
    def task(self):
        if len(self.tasks) > 0:
            return self.tasks[0]
        else:
            return -1

    @property
    def tasks(self):
        return list(self._active_tasks)

    @property
    def skills(self):
        """
        @brief Return the list of available skills
        """
        return self.get_skill_list(update=False)

    def shutdown(self):
        """
        @brief Unregister subscribers (note: deleting the instance without calling shutdown will leave callbacks active)
        """
        self._node.destroy_subscription(self._monitor_sub)
        # self._node.destroy_client(self._tick_rate_sub)

    def print_state(self):
        temp = "Skills: { "
        for c in self.get_skill_list():
            temp += c
            temp += ", "
        temp += "}"
        return temp

    def set_debug(self, state):
        """
        @brief Set skill manager debug mode on/off (publish more/less info about skill execution)
        @param state true=on, false=off
        """
        self._set_debug.publish(Bool(data=state))

    def get_skill_list(self, update=False):
        if update or not self._skill_list:
            msg = srvs.ResourceGetDescriptions.Request()
            res = self.call(self._get_skills, msg)
            self._skill_list = dict()
            if not res:
                log.error("[{}]".format(self.__class__.__name__), "Can t retrieve skills.")
            else:
                for c in res.list:
                    self._skill_list[c.name] = SkillHolder(self.name, c.type, c.name, utils.deserializeParamMap(c.params), available_for_planning=c.available_for_planning)
        return self._skill_list

    def get_skill(self, name):
        if not self._skill_list:
            self.get_skill_list()
        return self._skill_list[name]

    def execute(self, execution_id=-1, skill_list=None, action=srvs.SkillCommand.Request().START):
        """
        @brief Execute a list of skills
        """
        msg = srvs.SkillCommand.Request()
        msg.action = action
        msg.author = self._author
        msg.execution_id = execution_id
        if skill_list is not None:
            for s in skill_list:
                msg.skills.append(s.toMsg())
        res = self.call(self._skill_exe_client, msg)
        if res is None:
            return -1
        if not res.ok:
            log.error("", "Can t execute task.")
            return -1
        return res.execution_id

    def tick_once(self, execution_id=-1, skill_list=None):
        """
        @brief Tick behavior tree once
        """
        return self.execute(execution_id, skill_list, srvs.SkillCommand.Request().TICK_ONCE)

    def preempt_one(self, execution_id=None):
        """
        @brief Stop one task
        """
        msg = srvs.SkillCommand.Request()
        msg.action = msg.PREEMPT
        msg.author = self._author
        if not self.tasks:
            return False
        if execution_id is None:
            execution_id = self.task
        msg.execution_id = execution_id
        res = self.call(self._skill_exe_client, msg)
        if res is None:
            return False
        elif not res.ok:
            log.error("", "Can t stop task " + execution_id)
            return False
        return True

    def preempt_all(self):
        """
        @brief Stop all tasks
        """
        return self.preempt_one(-1)

    def pause_one(self, execution_id=None):
        """
        @brief Pause ticking
        """
        msg = srvs.SkillCommand.Request()
        msg.action = msg.PAUSE
        msg.author = self._author
        if not self.tasks:
            return False
        if execution_id is None:
            execution_id = self.task
        msg.execution_id = execution_id
        res = self.call(self._skill_exe_client, msg)
        if res is None:
            return False
        elif not res.ok:
            log.error("", "Can t stop tasks.")
            return False
        return True

    def pause_all(self):
        """
        @brief Pause ticking
        """
        return self.pause_one(-1)

    def set_monitor_cb(self, cb):
        """
        @brief Set an external callback on skill execution feedback
        """
        self._monitor_cb = cb

    def get_tick_rate(self):
        """
        @brief Get the skill manager tick rate. Not working on ROS2
        """
        return -1
        # rate_info = self._tick_rate.get_hz()
        # if rate_info is None:
        #     return 0
        # return rate_info[0]

    def reset_tick_rate(self):
        """
        @brief Reset the tick rate information
        """
        pass

    def _progress_cb(self, msg):
        root = [r for r in msg.progress if r.type.find("Root") >= 0]
        if root:
            self._active_tasks.add(int(root[-1].task_id))
            if abs(root[-1].progress_code) == 1:
                self._active_tasks.remove(int(root[-1].task_id))
        if self._monitor_cb:
            self._monitor_cb(msg)

    def call(self, service, msg):
        future = service.call_async(msg)
        if self._allow_spinning:
            # log.debug("Service call to {} with spinning".format(service.srv_name)) # Commented out until log levels work
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.)
            while not future.done():
                log.warn("[{}]".format(self.__class__.__name__), "Waiting for reply from service {} ...".format(service.srv_name))
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.)
        else:
            while rclpy.ok() and not future.done():
                pass
        return future.result()
