import skiros2_common.tools.logger as log
from skiros2_skill.ros.skill_manager_interface import SkillManagerInterface
from discovery_interface import DiscoveryInterface


class SkillLayerInterface(DiscoveryInterface):
    def __init__(self, author="unknown"):
        self._author = author
        self._agents = dict()
        self._new_changes = False
        self._active_sm = set()
        self.set_monitor_cb(None)
        self.init_discovery("skill_managers", self._on_active, self._on_inactive)

    def get_agent(self, agent=None):
        """
        @brief Return a skill manager interface
        @param agent, the key of the agent
        """
        if isinstance(agent, str):
            return self._agents[agent]
        return self._agents[agent.getProperty("skiros:SkillMgr").value]

    @property
    def agent(self):
        """
        @brief Return the first available skill manager
        """
        return self._agents.itervalues().next()

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
    def has_active_agents(self):
        return bool(self._active_sm)

    def set_monitor_cb(self, cb):
        """
        @brief Set an external callback on skill execution feedback
        """
        self._monitor_cb = cb

    def set_debug(self, state):
        """
        @brief Set skill managers in debug mode
        """
        for a in self.agents.values():
            a.set_debug(state)

    def _on_active(self, name):
        log.info("[SkillLayerInterface]", "New skill manager detected: {}".format(name))
        self._agents[name] = SkillManagerInterface(name, self._author)
        self._agents[name].set_monitor_cb(self._progress_cb)
        self._new_changes = True

    def _on_inactive(self, name):
        log.info("[SkillLayerInterface]", "Skill manager {} went down.".format(name))
        self._agents[name].shutdown()
        del self._agents[name]
        self._new_changes = True

    def _progress_cb(self, msg):
        if msg.type.find("Root") >= 0:
            if abs(msg.progress_code) == 0:
                self._active_sm.add(msg.robot)
            else:
                try:
                    self._active_sm.remove(msg.robot)
                except Exception:
                    pass
        if self._monitor_cb:
            self._monitor_cb(msg)

    def print_state(self):
        for k, a in self._agents.iteritems():
            print k + ":" + a.printState()
