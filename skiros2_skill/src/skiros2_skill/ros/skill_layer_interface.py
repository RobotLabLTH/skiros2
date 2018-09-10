#################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Francesco Rovida
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################################

import rospy

import skiros2_msgs.msg as msgs
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
        self.init_discovery("skill_managers", self._discovery_cb)

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
        if tid>=0:
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
            self._active_sm.remove(skill_mgr)
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

    def _discovery_cb(self, msg):
        #TODO: make a call for changes
        if msg.state==msg.ACTIVE and not self._agents.has_key(msg.name):
            log.info("[SkillLayerInterface]", "New skill manager detected: {}".format(msg.name))
            self._agents[msg.name] = SkillManagerInterface(msg.name)
            self._agents[msg.name].set_monitor_cb(self._progress_cb)
            self._new_changes = True
        elif msg.state==msg.INACTIVE and self._agents.has_key(msg.name):
            log.info("[SkillLayerInterface]", "Skill manager {} went down.".format(msg.name))
            del self._agents[msg.name]
            self._new_changes = True

    def _progress_cb(self, msg):
        if msg.label.find("task")>=0 and msg.progress_message=="End" and msg.state!=msg.IDLE:
            try:
                self._active_sm.remove(msg.robot)
            except Exception:
                pass
        if self._monitor_cb:
            self._monitor_cb(msg)

    def print_state(self):
        for k, a in self._agents.iteritems():
            print k + ":" + a.printState()