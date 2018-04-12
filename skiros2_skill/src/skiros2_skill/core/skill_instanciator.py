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
from skiros2_common.core.abstract_skill import SkillDescription, State

class SkillInstanciator:
    def __init__(self, wmi):
        self._available_descriptions={}
        self._available_instances=defaultdict(list)
        self._wm = wmi

    def _makeDescription(self, skill):
        to_ret = SkillDescription()
        to_ret.setDescription(*skill.getDescription())
        return to_ret

    def addDescription(self, skill):
        self._available_descriptions[skill._type] = skill

    def addInstance(self, skill):
        skill.init(self._wm, self)
        if not skill._type in self._available_descriptions:
            self._available_descriptions[skill._type] = self._makeDescription(skill)
        self._available_instances[skill._type].append(skill)

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
            skill.setDescription(self._available_descriptions[skill._type])
        else:
            log.error("assignDescription", "No instances of type {} found.".format(skill._type))

    def getInstances(self, ptype):
        return self._available_instances[ptype]

    def assignInstance(self, skill):
        """
        @brief Assign an instance to an abstract skill.

        If an instance with same label is not found, assign the first instance of the type.
        """
        to_set = None
        first_cycle = True
        for p in self._available_instances[skill._type]:
            if first_cycle: #TODO: removed temporarly-> and not p.hasState(State.Running)
                first_cycle = False
                to_set = p
            if p._label == skill._label:
                to_set = p
        if to_set != None:
            skill.setInstance(to_set)
        else:
            log.error("assignInstance", "No instances of type {} found.".format(skill._type))

    def printState(self, verbose=True, filter_type=""):
        s = 'Descriptions:\n'
        for t, p in self._available_descriptions.iteritems():
            if p._type==filter_type or filter_type=="":
                s += p.printInfo(verbose)
                s += '\n'
        s += '\nInstances:\n'
        for k, l in self._available_instances.iteritems():
            for p in l:
                if p._type==filter_type or filter_type=="":
                    s += p.printInfo(verbose)
                    s += '\n'
        return s

