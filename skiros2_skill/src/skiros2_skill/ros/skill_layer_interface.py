import skiros2_common.tools.logger as log
from skiros2_skill.ros.skill_manager_interface import SkillManagerInterface
from discovery_interface import DiscoveryInterface


class SkillLayerInterface(DiscoveryInterface):
    def __init__(self, author="unknown"):
        self._author = author
        self._agents = dict()
        self._new_changes = False
        self._active_sm = list()
        self.set_monitor_cb(None)
        self.init_discovery("skill_managers", self._on_active, self._on_inactive)

    def get_agent(self, agent):
        if isinstance(agent, str):
            return self._agents[agent]
        else:
            return self._agents[agent.getProperty("skiros:SkillMgr").value]

    @property
    def agents(self):
        """
        @brief Return the list of available skill managers
        """
        return self._agents

    @property
    def has_changes(self):
        if self._new_changes:
            self._new_changes = False
            return True
        return False

    @property
    def has_active_tasks(self):
        return bool(self._active_sm)

    def execute(self, skill_mgr, skill_list):
        """
        @brief Start a task execution
        """
        tid = self.get_agent(skill_mgr).execute(skill_list, self._author)
        if tid >= 0:
            self._active_sm.insert(0, skill_mgr)
        return tid

    def preempt_one(self, skill_mgr=None, tid=None):
        """
        @brief Stop a task execution
        @param skill_mgr manager id. If left blank, invoke the last
        @param tid execution id. If left blank, stops the last execution
        """
        if not self.has_active_tasks:
            return False
        if skill_mgr is None:
            skill_mgr = self._active_sm[0]
        if self.get_agent(skill_mgr).preempt(self._author, tid):
            return True
        return False

    def preempt_all(self):
        """
        @brief Stop all tasks execution
        """
        for a in self._agents.values():
            a.preempt_all(self._author)

    def set_monitor_cb(self, cb):
        """
        @brief Set an external callback on skill execution feedback
        """
        self._monitor_cb = cb

    def _on_active(self, name):
        log.info("[SkillLayerInterface]", "New skill manager detected: {}".format(name))
        self._agents[name] = SkillManagerInterface(name)
        self._agents[name].set_monitor_cb(self._progress_cb)
        self._new_changes = True

    def _on_inactive(self, name):
        log.info("[SkillLayerInterface]", "Skill manager {} went down.".format(name))
        self._agents[name].shutdown()
        del self._agents[name]
        self._new_changes = True

    def _progress_cb(self, msg):
        if msg.type.find("Root") >= 0 and abs(msg.progress_code) == 1:
            try:
                self._active_sm.remove(msg.robot)
            except Exception:
                pass
        if self._monitor_cb:
            self._monitor_cb(msg)

    def print_state(self):
        for k, a in self._agents.iteritems():
            print k + ":" + a.printState()
