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
import skiros2_msgs.srv as srvs
from std_srvs.srv import SetBool, SetBoolResponse
from skiros2_world_model.core.ontology_rdflib import Ontology
from skiros2_common.tools.time_keeper import TimeKeeper
import skiros2_common.ros.utils as utils
from threading import Lock
from pyparsing import ParseException

class OntologyServer(object):
    def __init__(self, anonymous=False):
        rospy.init_node("ontology", anonymous=anonymous)
        self._verbose = rospy.get_param('~verbose', False)
        self._ontology =  Ontology()
        self.initOntologyServices()

    def initOntologyServices(self):
        self._times = TimeKeeper()
        self._mutex = Lock()
        self._mutex_srv = rospy.Service('~lock', SetBool, self._lockCb)
        self._query = rospy.Service('~ontology/query', srvs.WoQuery, self._woQueryCb)
        self._modify = rospy.Service('~ontology/modify', srvs.WoModify, self._woModifyCb)

    def _lockCb(self, msg):
        """
        @brief Can be used to get sync access to server
        """
        if msg.data:
            self._mutex.acquire()
        else:
            try:
                self._mutex.release()
            except:
                return SetBoolResponse(False, "Mutex already unlocked.")
        return SetBoolResponse(True, "Ok")

    def _woQueryCb(self, msg, trial=0):
        to_ret = srvs.WoQueryResponse()
        try:
            with self._times:
                for s in self._ontology.query(msg.query_string):
                    temp = ""
                    for r in s:
                        if msg.cut_prefix:
                            temp += self._ontology.uri2lightstring(r)
                        else:
                            temp += r.n3()
                        if len(s)>1:
                            temp += " "
                    to_ret.answer.append(temp)
            if self._verbose:
                log.info("[WoQuery]", "Query: {}. Answer: {}. Time: {:0.3f} sec".format(msg.query_string, to_ret.answer, self._times.getLast()))
        except (AttributeError, ParseException) as e:
            #TODO: Understand what is going wrong here. For now just retry the query a couple of times seems to cover the bug
            log.error("[WoQuery]", "Parse error with following query: {}. Error: {}".format(msg.query_string, e))
            if trial<2:
                trial += 1
                log.info("[WoQuery]", "Retring query {}.".format(trial))
                return self._woQueryCb(msg, trial)
        return to_ret

    def _woModifyCb(self, msg):
        with self._times:
            for s in msg.statements:
                if s.value:
                    self._ontology.addRelation(utils.msg2relation(s.relation), msg.author, is_relation=False)
                else:
                    self._ontology.removeRelation(utils.msg2relation(s.relation), msg.author, is_relation=False)
        if self._verbose:
            log.info("[WoModify]", "Done in {} sec".format(self._times.getLast()))
        return srvs.WoModifyResponse(True)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = OntologyServer()
    node.run()
