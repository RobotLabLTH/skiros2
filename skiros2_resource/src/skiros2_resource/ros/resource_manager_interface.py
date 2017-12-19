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

import sys
import rospy
import skiros2_common.tools.logger as log
import skiros2_msgs.msg as msgs
import skiros2_msgs.srv as srvs
import skiros2_common.ros.utils as utils
from skiros2_common.core.abstract_skill import State
from multiprocessing import Lock, Event

class ResourceManagerInterface:
    def __init__(self, name):
        self._command = rospy.ServiceProxy(name+'/command', srvs.ResourceCommand)
        self._set_params = rospy.ServiceProxy(name+'/set_params', srvs.ResourceSetParams)
        self._get_description = rospy.ServiceProxy(name+'/get_descriptions', srvs.ResourceGetDescriptions)
        self._monitor = rospy.Subscriber(name+'/monitor', msgs.ResourceMonitor, self._monitor_cb)
        self._msg_lock = Lock()
        self._msg_rec = Event()
        self._resource_list = {}
        self._running_resources = {}
        self._results = {}
        self.getResourceList(True)

    def getResourceList(self, update=False):
        if update:
            for c in self.getDescriptions():
                self._resource_list[c.name] = c
        return self._resource_list

    def printState(self):
        temp = "Resources: {"
        for c in self.getResourceList():
            temp += c
            temp += ", "
        temp += "}"
        return temp

    def _monitor_cb(self, msg):
        """
        Save the result in self._results[name] and remove the resource from self._running_resources
        """
        name = msg.resource.name
        if self._running_resources.has_key(name):
            with self._msg_lock:
                self._results[name] = msg
                del self._running_resources[name]
                self._msg_rec.set()

    def isRunning(self, name):
        if not self._running_resources.has_key(name):
            return False
        return True
        
    def waitResult(self, name):
        if self._running_resources.has_key(name):
            while not self._results.has_key(name) or self._running_resources.has_key(name):
                self._msg_rec.wait()
            
    def getResult(self, name):
        if self._results.has_key(name):
            params = utils.deserializeParamMap(self._results[name].resource.params)
            del self._results[name]
            return params

    def setParams(self, name, params):
        msg = srvs.ResourceSetParamsRequest()
        msg.resource_name = name
        msg.param_map = utils.serializeParamMap(params)
        res = self.__call(self._set_params, msg)
        if not res:
            log.error("[ResourceMgrInterface]", "Error in service call. Can t parametrize Resource " + name)
            return False
        return res.ok

    def getDescriptions(self):
        """
        Return a list of available resources on the server
        """
        return self.__call(self._get_description, srvs.ResourceGetDescriptionsRequest()).list

    def preempt(self, name, author):
        msg = srvs.ResourceCommandRequest()
        msg.command = msg.PREEMPT;
        msg.author = author;
        msg.resource_name = name;
        res = self.__call(self._command, msg)
        if not res:
            log.error("ResourceMgrInterface", "Can t stop Resource " + name)
            return State.Failure
        return State(res.state)
        
    def tick(self, name, author):
        while self._running_resources.has_key(name):
            self._msg_rec.wait()
        msg = srvs.ResourceCommandRequest()
        msg.command = msg.TICK;
        msg.author = author;
        msg.resource_name = name;
        self._running_resources[name] = msgs.ResourceMonitor()
        res = self.__call(self._command, msg)
        if not res:
            log.error("ResourceMgrInterface", "Can t execute Resource " + name)
            return State.Failure
        return State(res.state)

    def reset(self, name, author):
        while self._running_resources.has_key(name):
            self._msg_rec.wait()
        msg = srvs.ResourceCommandRequest()
        msg.command = msg.RESET
        msg.author = author
        msg.resource_name = name
        self._running_resources[name] = msgs.ResourceMonitor()
        res = self.__call(self._command, msg)
        if not res:
            log.error("Can t reset Resource " + name)
            return State.Failure
        return State(res.state)

    def __call(self, service, msg):
        try:
            resp1 = service(msg)
            return resp1
        except rospy.ServiceException, e:
            log.error("[ResourceMgrInterface]", "Service call failed: %s"%e)
            return
