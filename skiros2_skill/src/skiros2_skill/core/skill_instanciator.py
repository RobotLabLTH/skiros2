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

from collections import defaultdict
import skiros2_common.tools.logger as log
from skiros2_common.core.abstract_skill import State
from copy import deepcopy

class SkillInstanciator:
    def __init__(self, wmi):
        self._available_descriptions={}
        self._available_instances=defaultdict(list)
        self._wm = wmi

    def addDescription(self, skill):
        self._available_descriptions[skill._type] = skill

    def addInstance(self, skill):
        skill.init(self._wm, self)
        if not skill._type in self._available_descriptions:
            self._available_descriptions[skill.type] = skill._description
        self._available_instances[skill.type].append(skill)

    def expandAll(self):
        for _, ps in self._available_instances.iteritems():
            for p in ps:
                p.expand(p)

    def assignDescription(self, skill):
        """
        Assign a description to an abstract skill.
        """
        if skill._type in self._available_descriptions:
            skill.init(self._wm)
            skill.setDescription(deepcopy(self._available_descriptions[skill.type]))
        else:
            log.error("assignDescription", "No instances of type {} found. Debug: {}".format(skill.type, self._available_descriptions.keys()))

    def getInstances(self, ptype):
        return self._available_instances[ptype]

    def duplicate_instance(self, instance):
        """
        @brief Add a new instance of the skill in the available list
        """
        new = instance.__class__()
        new.init(self._wm, self)
        self._available_instances[new.type].append(new)
        return new

    def assignInstance(self, skill, ignore_list=list()):
        """
        @brief Assign an instance to an abstract skill.

        If an instance with same label is not found, assign the last instance of the type.
        """
        to_set = None
        for p in self._available_instances[skill.type]:
            if (p.label == skill.label or skill.label=="") and p.label not in ignore_list:
                to_set = p
                if not p.hasState(State.Running):#The skill is available, just go forward
                    break
        if to_set is not None:
            if to_set.hasState(State.Running):#The skill instance is busy, create a new one
                to_set = self.duplicate_instance(to_set)
            skill.setInstance(to_set)
        else:
            log.error("assignInstance", "No instance of type {} found.".format(skill.type))
            return False
        return True

    def printState(self, verbose=True, filter_type=""):
        s = 'Descriptions:\n'
        for t, p in self._available_descriptions.iteritems():
            if p.type==filter_type or filter_type=="":
                s += p.printInfo(verbose)
                s += '\n'
        s += '\nInstances:\n'
        for k, l in self._available_instances.iteritems():
            for p in l:
                if p.type==filter_type or filter_type=="":
                    s += p.printInfo(verbose)
                    s += '\n'
        return s

