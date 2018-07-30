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
    def __init__(self, skill_monitor_cb=None):
        """
        Initialize agents list
        """
        self._agents = {}
        self._new_changes = True
        self._monitor_sub = rospy.Subscriber("skill_managers/monitor", msgs.SkillProgress, self._progress_cb)
        self._monitor_cb = None
        self.init_discovery("skill_managers", self._discovery_cb)
        if skill_monitor_cb:
            self.set_monitor_cb(skill_monitor_cb)

    def getAgent(self, agent):
        if isinstance(agent, str):
            return self._agents[agent]
        else:
            return self._agents[agent.getProperty("skiros:SkillMgr").value]

    def printState(self):
        for k, a in self._agents.iteritems():
            print k + ":" + a.printState()

    def hasChanges(self):
        if self._new_changes:
            self._new_changes = False
            return True
        return False

    def set_monitor_cb(self, cb):
        self._monitor_cb = cb

    def _discovery_cb(self, msg):
        if msg.state==msg.ACTIVE and not self._agents.has_key(msg.name):
            log.info("[SkillLayerInterface]", "New skill manager detected: {}".format(msg.name))
            self._agents[msg.name] = SkillManagerInterface(msg.name)
            self._new_changes = True
        elif msg.state==msg.INACTIVE and self._agents.has_key(msg.name):
            log.info("[SkillLayerInterface]", "Skill manager {} went down.".format(msg.name))
            del self._agents[msg.name]
            self._new_changes = True

    def _progress_cb(self, msg):
        if self._monitor_cb:
            self._monitor_cb(msg)
