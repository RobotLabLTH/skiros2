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
import skiros2_common.tools.logger as log
import skiros2_common.core.params as params
import skiros2_common.ros.utils as utils
from skiros2_common.core.primitive import PrimitiveBase 
from skiros2_common.tools.plugin_loader import PluginLoader
import skiros2_msgs.msg as msgs
import skiros2_msgs.srv as srvs

def resource2msg(resource):
    msg = msgs.ResourceDescription()
    msg.type = resource._type
    msg.name = resource._label
    msg.params = utils.serializeParamMap(resource._params.getParamMap())   
    return msg

class ResourceManager(object):
    """
    At boot:
        -load resources defined in ros params 'libraries_list' and 'resource_list'
    Main roles:
        -propagate resource info with service /get_descriptions
        -parametrize resources with service /set_params
        -execute resources with service /command
        -print resource states on topic /monitor 
    """    
    def __init__(self, anonymous=False):
        rospy.init_node("resource_mgr", anonymous=anonymous)
        self._verbose = rospy.get_param('~verbose', True)
        self._command = rospy.Service('~command', srvs.ResourceCommand, self._commandCb)
        self._setparams = rospy.Service('~set_params', srvs.ResourceSetParams, self._setParamsCb)
        self._getparams = rospy.Service('~get_descriptions', srvs.ResourceGetDescriptions, self._getDescriptionsCb)
        self._monitor = rospy.Publisher("~monitor", msgs.ResourceMonitor, queue_size=20)
        self._resource_map = {}
        self._active_map = {}
        self._initResources()
        #rospack = rospkg.RosPack()
        #path=rospack.get_path('skiros2_common')
        
    def _initResources(self, ):
        """
        Load plugins and store them in a dictionary 'self._resource_map'
        """
        self.plug_loader = PluginLoader()
        #Load plugins descriptions
        for r in rospy.get_param('~libraries_list', []):
            self.plug_loader.load(r, PrimitiveBase)
        #Initialize desired plugins
        for r in rospy.get_param('~resource_list', []):
            self._resource_map[r] = self.plug_loader.getPluginByName(r)() #"TestResource"
            if not self._resource_map[r].init():
                log.error("", "Resource {} failed to initialize".format(r))
            self._resource_map[r].reset()
            print self._resource_map[r].printInfo(True)

    def _output(self, name, author):
        """
        Publish on ~/monitor topic
        """
        monitorMsg = msgs.ResourceMonitor()
        monitorMsg.owner = author
        monitorMsg.header.stamp = rospy.Time.now()
        monitorMsg.resource.type = self._resource_map[name]._type
        monitorMsg.resource.name = name
        monitorMsg.resource.params = utils.serializeParamMap(self._resource_map[name]._params.getParamMap())      
        monitorMsg.state = self._resource_map[name].getState().name
        monitorMsg.progress_code = self._resource_map[name]._progress_code
        monitorMsg.progress_description = self._resource_map[name]._progress_msg
        self._monitor.publish(monitorMsg)
        return monitorMsg

    def _commandCb(self, msg):
        """
        Called when receiving a command on ~/command
        """
        answer = srvs.ResourceCommandResponse()
        if msg.command == msg.TICK:
            self._resource_map[msg.resource_name].tick()
        elif msg.command==msg.RESET:
            self._resource_map[msg.resource_name].reset()
        elif msg.command==msg.PREEMPT:
            self._resource_map[msg.resource_name].preempt()
        monitorMsg = self._output(msg.resource_name, msg.author)
        answer.state = monitorMsg.state
        if self._verbose and monitorMsg.progress_description!="":
            log.info("[{}]".format(msg.resource_name), "{} State: {} Progress: {}".format(msg.command, self._resource_map[msg.resource_name]._state.name, self._resource_map[msg.resource_name].printProgress()))
        return answer
        
    def _setParamsCb(self, msg):
        """
        Called when receiving a command on ~/set_params
        """
        self._resource_map[msg.resource_name]._params.specifyParams(params.ParamHandler(utils.deserializeParamMap(msg.param_map)), False)
        return srvs.ResourceSetParamsResponse(True, "")
        
    def _getDescriptionsCb(self, msg):
        """
        Called when receiving a command on ~/get_descriptions
        """
        to_ret = srvs.ResourceGetDescriptionsResponse()
        for k, r in self._resource_map.iteritems():
            to_ret.list.append(resource2msg(r))
        return to_ret

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ResourceManager()
    node.run()
