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
import skiros2_msgs.srv as srvs
import skiros2_common.ros.utils as utils
from skiros2_skill.ros.utils import SkillHolder
import skiros2_common.tools.logger as log
from multiprocessing import Lock, Event


class SkillManagerInterface:
    def __init__(self, manager_name):
        self._skill_mgr_name = manager_name
        self._active_tasks = list()
        self._module_list = dict()
        self._skill_list = dict()
        self._msg_lock = Lock()
        self._msg_rec = Event()
        rospy.wait_for_service(self._skill_mgr_name + '/get_skills')
        self._skill_exe_client = rospy.ServiceProxy(self._skill_mgr_name + '/command', srvs.SkillCommand)
        self._get_skills = rospy.ServiceProxy(self._skill_mgr_name + '/get_skills', srvs.ResourceGetDescriptions)
        self._monitor_sub = rospy.Subscriber(self._skill_mgr_name + '/monitor', msgs.SkillProgress, self._progress_cb)
        self._monitor_cb = None
        self.get_skill_list(True)

    @property
    def name(self):
        return self._skill_mgr_name

    @property
    def active_tasks(self):
        return self._active_tasks

    def print_state(self):
        temp = "Skills: { "
        for c in self.get_skill_list():
            temp += c
            temp += ", "
        temp += "}"
        return temp

    def get_skill_list(self, update=False):
        if update or not self._skill_list:
            msg = srvs.ResourceGetDescriptionsRequest()
            res = self.call(self._get_skills, msg)
            self._skill_list = dict()
            if not res:
                log.error("[{}]".format(self.__class__.__name__), "Can t retrieve skills.")
                return -1
            for c in res.list:
                self._skill_list[c.name] = SkillHolder("", c.type, c.name, utils.deserializeParamMap(c.params))
        return self._skill_list

    def get_skill(self, name):
        return self._skill_list[name]

    def execute(self, skill_list, author):
        msg = srvs.SkillCommandRequest()
        msg.action = msg.START;
        msg.author = author;
        for s in skill_list:
            msg.skills.append(s.toMsg())
        res = self.call(self._skill_exe_client, msg)
        if res is None:
            return -1
        if not res.ok:
            log.error("Can t execute task ")
            return -1
        self._active_tasks.append(res.execution_id)
        return res.execution_id

    def preempt(self, author, execution_id=None):
        msg = srvs.SkillCommandRequest()
        msg.action = msg.PREEMPT;
        msg.author = author;
        if not self.active_tasks:
            return False
        if execution_id is None:
            execution_id = self.active_tasks[-1]
        msg.execution_id = execution_id
        res = self.call(self._skill_exe_client, msg)
        if res is None:
            return False
        elif not res.ok:
            log.error("Can t stop task " + execution_id)
            return False
        self._active_tasks.remove(execution_id)
        return True

    def preempt_all(self, author):
        msg = srvs.SkillCommandRequest()
        msg.action = msg.PREEMPT;
        msg.author = author;
        msg.execution_id = -1
        res = self.call(self._skill_exe_client, msg)
        if res is None:
            return False
        elif not res.ok:
            log.error("Can t stop tasks.")
            return False
        self._active_tasks = list()
        return True

    def set_monitor_cb(self, cb):
        """
        @brief Set an external callback on skill execution feedback
        """
        self._monitor_cb = cb

    def _progress_cb(self, msg):
        if msg.label.find("task")>=0 and msg.progress_message=="End" and msg.state!=msg.IDLE:
            try:
                self._active_tasks.remove(int(msg.task_id))
            except Exception:
                pass
        if self._monitor_cb:
            self._monitor_cb(msg)

    def call(self, service, msg):
        try:
            resp1 = service(msg)
            return resp1
        except rospy.ServiceException, e:
            log.error("[call]", "Service call failed: %s"%e)
            return
