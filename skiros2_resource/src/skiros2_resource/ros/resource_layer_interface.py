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
from resource_manager_interface import ResourceManagerInterface

class ResourceLayerInterface:
    """
    Interface to multiple resource managers
    """
    def __init__(self, commander_name=""):
        if commander_name=="":
            self._author = rospy.get_name()
        self._managers = {}
        self.updateManagersList()

    def getAvailableResources(self):
        """
        Return list of pairs (ManagerName, ResourceDescriptionMsg)
        """
        to_ret = []
        for key, mgr in self._managers.iteritems():
            for _, r in mgr.getResourceList().iteritems():
                to_ret.append((key, r))
        return to_ret
        
    def getManager(self, name):
        return self._managers[name]

    def printState(self):
        for k, a in self._managers.iteritems():
            print k + ": " + a.printState()

    def updateManagersList(self):
        """
        Update managers list
        """
        #Retrieve
        list = [topic.replace('/monitor', '') for topic, type in rospy.get_published_topics() if type=='skiros2_msgs/ResourceMonitor']
        #Remove obsoletes
        for key, mgr in self._managers.iteritems():
            if not key in list:
                del self._managers
        #Add news
        for name in list:
            if not self._managers.has_key(name):
                self._managers[name] = ResourceManagerInterface(name)
            
    def isRunning(self, robot, name):    
        mgr = self.getManager(robot)
        return mgr.isRunning(name)    
        
    def getState(self, robot, name):    
        mgr = self.getManager(robot)
        return mgr.getState(name)     
        
    def getResult(self, robot, name, wait=True):    
        mgr = self.getManager(robot)
        if wait: 
            mgr.waitResult(name)
        return mgr.getResult(name)             
        
    def start(self, robot, name, params):    
        mgr = self.getManager(robot)
        mgr.setParams(name, params)
        return mgr.tick(name, self._author)
        
    def reset(self, robot, name):    
        mgr = self.getManager(robot)
        return mgr.reset(name, self._author)
        
    def preempt(self, robot, name):    
        mgr = self.getManager(robot)
        return mgr.preempt(name, self._author)
        
    def tick(self, robot, name, params):    
        mgr = self.getManager(robot)
        mgr.setParams(name, params)
        return mgr.tick(name, self._author)