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
    def __init__(self, wmi, robot, monitor_cb=None):
        self._wmi = wmi
        self._robot = robot
        self._robot_uri = robot._label
        self._monitored_tasks = dict()
        self._module_list = dict()
        self._skill_list = dict()
        self.init(monitor_cb)
        self._msg_lock = Lock()
        self._msg_rec = Event()

    def init(self, monitor_cb):
        self._skill_mgr_name = self._robot.getProperty('skiros:SkillMgr').getValue()
        rospy.wait_for_service(self._skill_mgr_name + '/get_skills')
        self._skill_exe_client = rospy.ServiceProxy(self._skill_mgr_name + '/command', srvs.SkillCommand)
        if monitor_cb:
            self._monitor_sub = rospy.Subscriber(self._skill_mgr_name + '/monitor', msgs.SkillProgress, monitor_cb)
        self._get_skills = rospy.ServiceProxy(self._skill_mgr_name + '/get_skills', srvs.ResourceGetDescriptions)
        self._skill_set = self._wmi.getSubClasses("Skill")
        self.getSkillList(True)

    def printState(self):
        temp = "Skills: { "
        for c in self.getSkillList():
            temp += c
            temp += ", "
        temp += "}"
        return temp

    def _smMonitorCB(self, msg):
        #with self._mutex:
        print msg
        execution_id = msg.owner
        if self._monitored_tasks.has_key(execution_id):
            with self._msg_lock:
                self._monitored_tasks[execution_id] = msg
                self._msg_rec.set()

    def waitResult(self, execution_id, timeout=None):
        if not self._monitored_tasks.has_key(execution_id):
            log.warn("SkillManagerInterface.waitResult", "No task {} in list ".format(execution_id))
            return False
        if not self._monitored_tasks[execution_id]:
            self._msg_rec.wait(timeout)
        with self._msg_lock:
            while not self._monitored_tasks[execution_id].state in ['failure', 'preempted', 'success']:
                self._msg_lock.release()
                if not self._msg_rec.wait(timeout):
                    pass #Unreliable: it returns false if it fails to lock the event
                    #log.warn("SkillManagerInterface.waitResult", "Hit timeout {}".format(timeout))
                    #return False
                self._msg_lock.acquire()
                self._msg_rec.clear()
        if(self._monitored_tasks[execution_id].status == 'success'):
            return True
        else:
            log.warn("SkillManagerInterface.waitResult", "Final status {}".format(self._monitored_tasks[execution_id].status))
            return False

    def getResult(self, execution_id):
        if not self._monitored_tasks.has_key(execution_id):
            return False
        return utils.deserializeParamMap(self._monitored_tasks[execution_id].resource.status)

    def getSkillList(self, update=False):
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

    def getSkill(self, name):
        return self._skill_list[name]

    def execute(self, skill_list, author):
        msg = srvs.SkillCommandRequest()
        msg.action = msg.START;
        msg.author = author;
        for s in skill_list:
            msg.skills.append(s.toMsg())
        res = self.call(self._skill_exe_client, msg)
        if not res.ok:
            log.error("Can t execute task ")
            return -1
        self._monitored_tasks[res.execution_id] = None
        return res.execution_id

    def preempt(self, execution_id, author):
        msg = srvs.SkillCommandRequest()
        msg.action = msg.PREEMPT;
        msg.author = author;
        msg.execution_id = execution_id
        res = self.call(self._skill_exe_client, msg)
        if not res.ok:
            log.error("Can t stop task " + execution_id)
            return False
        return True

    def call(self, service, msg):
        try:
            resp1 = service(msg)
            return resp1
        except rospy.ServiceException, e:
            log.error("[call]", "Service call failed: %s"%e)
            return
