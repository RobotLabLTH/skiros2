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

from multiprocessing.dummy import Process

class DiscreteReasoner(object):
    def init(self, wmi):
        """
        Set an interface to the world model
        """
        self._stop_requested = False
        self._thread = None
        self._wmi = wmi

    def parse(self, element, action):
        """ Parse the action (add, remove,update) [element] """
        raise NotImplementedError("Not implemented in abstract class")

    @property
    def stopRequested(self):
        to_ret = self._stop_requested
        self._stop_requested = False
        return to_ret

    def stop(self):
        """ Request to stop the reasoner """
        self._stop_requested = True

    def execute(self):
        self._thread = Process(target=self.run)
        self._thread.start()

    def run(self):
        """ Run the reasoner daemon on the world model """
        raise NotImplementedError("Not implemented in abstract class")

    def addProperties(self, element):
        if not element.hasProperty("skiros:DiscreteReasoner", self.__class__.__name__):
            element.setProperty("skiros:DiscreteReasoner", self.__class__.__name__, is_list=True)
        self.onAddProperties(element)

    def onAddProperties(self, element):
        """ Add default reasoner properties to the element """
        raise NotImplementedError("Not implemented in abstract class")

    def removeProperties(self, element):
        """ Remove default reasoner properties from the element """
        if element.hasProperty("skiros:DiscreteReasoner", self.__class__.__name__):
            element.getProperty("skiros:DiscreteReasoner").removeValue(self.__class__.__name__)
        self.onRemoveProperties(element)

    def onRemoveProperties(self, element):
        """ Remove default reasoner properties from the element """
        raise NotImplementedError("Not implemented in abstract class")

    def hasData(self, element, get_code):
        """ Return data from the element in the format indicated in get_code """
        raise NotImplementedError("Not implemented in abstract class")

    def getData(self, element, get_code):
        """ Return data from the element in the format indicated in get_code """
        raise NotImplementedError("Not implemented in abstract class")

    def setData(self, element, data, set_code):
        """ Convert user data to reasoner data and store it into given element """
        raise NotImplementedError("Not implemented in abstract class")

    def getAssociatedRelations(self):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")

    def getAssociatedProperties(self):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")

    def getAssociatedData(self):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")

    def computeRelations(self, sub, obj, with_metrics=False):
        """ TODO """
        raise NotImplementedError("Not implemented in abstract class")
