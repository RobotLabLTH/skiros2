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
import skiros2_common.ros.utils as utils
import skiros2_msgs.msg as msgs
import skiros2_msgs.srv as srvs
from skiros2_common.tools.plugin_loader import PluginLoader
from skiros2_common.core.discrete_reasoner import DiscreteReasoner
from skiros2_world_model.ros.ontology_server import OntologyServer
from skiros2_world_model.core.world_model import WorldModel, Element
import uuid
from time import sleep

class WorldModelServer(OntologyServer):
    def __init__(self, anonymous=False):
        self._monitor = None
        rospy.init_node("wm", anonymous=anonymous)
        rospy.on_shutdown(self._waitClientsDisconnection)
        self._verbose = rospy.get_param('~verbose', False)
        self._wm = WorldModel(self._verbose, self.wm_change_cb)
        self._ontology = self._wm
        self._plug_loader = PluginLoader()
        self._loadReasoners()
        #================Snapshot======================
        self._curr_snapshot = uuid.uuid4() # random UUID
        #self._snapshots_log = []
        #================ROS======================
        self._set_relation = rospy.Service('~scene/set_relation', srvs.WmSetRelation, self._wmSetRelCb)
        self._query_relations = rospy.Service('~scene/query_relations', srvs.WmQueryRelations, self._wmQueryRelCb)
        self._get = rospy.Service('~scene/get', srvs.WmGet, self._wmGetCb)
        self._modify = rospy.Service('~scene/modify', srvs.WmModify, self._wmModifyCb)
        self._get = rospy.Service('~scene/load_and_save', srvs.WmLoadAndSave, self._wmLoadAndSaveCb)
        self._monitor = rospy.Publisher("~monitor", msgs.WmMonitor, queue_size=20, latch=True)
        self.initOntologyServices()

    def _waitClientsDisconnection(self):
        if self._monitor:
            while self._monitor.get_num_connections()>0:
                sleep(0.1)

    def wm_change_cb(self, author, action, element):
        self._publishChange(author, action, [utils.element2msg(element)])

    def _publishChange(self, author, action, elements=None, relation=None):
        msg =  msgs.WmMonitor()
        msg.prev_snapshot_id = self._curr_snapshot.hex
        self._curr_snapshot = uuid.uuid4() # random UUID
        msg.snapshot_id = self._curr_snapshot.hex
        msg.stamp = rospy.Time.now()
        msg.author = author
        msg.action = action
        if elements:
            msg.elements = elements
        if relation:
            msg.relation = relation
        self._monitor.publish(msg)

    def _loadReasoners(self):
        """
        Load reasoner plugins
        """
        #Load plugins descriptions
        for package in rospy.get_param('~reasoners_pkgs', []):
            self._plug_loader.load(package, DiscreteReasoner)
        for p in self._plug_loader:
            self._wm.loadReasoner(p)

    def _wmLoadAndSaveCb(self, msg):
        with self._times:
            if msg.action==msg.SAVE:
                self._wm.saveScene(msg.filename)
            elif msg.action==msg.LOAD:
                self._wm.loadScene(msg.filename)
        if self._verbose:
            log.info("[wmLoadAndSave]", "{} file {}. Time: {:0.3f} secs".format(msg.action, msg.filename, self._times.getLast()))
        return srvs.WmLoadAndSaveResponse(True)

    def _wmQueryRelCb(self, msg):
        to_ret = srvs.WmQueryRelationsResponse()
        with self._times:
            to_ret.matches = [utils.relation2msg(x) for x in self._wm.getRelations(utils.msg2relation(msg.relation))]
        if self._verbose:
            log.info("[wmQueryRelation]", "Query: {} Answer: {}. Time: {:0.3f} secs".format(msg.relation, to_ret.matches, self._times.getLast()))
        return to_ret

    def _wmGetCb(self, msg):
        with self._times:
            to_ret = srvs.WmGetResponse()
            if msg.action == msg.GET:
                to_ret.elements.append(utils.element2msg(self._wm.getElement(msg.element.id)))
            elif msg.action == msg.GET_TEMPLATE:
                to_ret.elements.append(utils.element2msg(self._wm.getTemplateIndividual(msg.element.label)))
            elif msg.action == msg.GET_RECURSIVE:
                for _, e in self._wm.getRecursive(msg.element.id, msg.relation_filter, msg.type_filter).iteritems():
                    to_ret.elements.append(utils.element2msg(e))
            elif msg.action == msg.RESOLVE:
                for e in self._wm.resolveElements(utils.msg2element(msg.element)):
                    to_ret.elements.append(utils.element2msg(e))
        output = ""
        einput = utils.msg2element(msg.element)
        for e in to_ret.elements:
            output += "{} ".format(e.id)
        if self._verbose:
            log.info("[WmGet]", "Done {} [{}]. Answer: {}. Time: {:0.3f} secs".format(msg.action, einput, output, self._times.getLast()))
        to_ret.snapshot_id = self._curr_snapshot.hex
        return to_ret

    def _wmSetRelCb(self, msg):
        with self._times:
            if msg.value:
                temp = "+"
                self._wm.addRelation(utils.msg2relation(msg.relation), msg.author)
                self._publishChange(msg.author, "add", relation=msg.relation)
            else:
                temp = "-"
                self._wm.removeRelation(utils.msg2relation(msg.relation), msg.author)
                self._publishChange(msg.author, "remove", relation=msg.relation)
        if self._verbose:
            log.info("[wmSetRelCb]", "[{}] {} Time: {:0.3f} secs".format(temp, msg.relation, self._times.getLast()))
        return srvs.WmSetRelationResponse(True)

    def _wmModifyCb(self, msg):
        to_ret = srvs.WmModifyResponse()
        with self._times:
            if msg.action == msg.ADD:
                e_list = list()
                for e in msg.elements:
                    updated_e = self._wm.addElement(utils.msg2element(e), msg.author)
                    to_ret.ids.append(updated_e.id)
                    e_list.append(utils.element2msg(updated_e))
                self._publishChange(msg.author, "add", elements=e_list)
            elif msg.action == msg.UPDATE:
                e_list = list()
                for e in msg.elements:
                    self._wm.updateElement(utils.msg2element(e), msg.author)
                    to_ret.ids.append(e.id)
                    e_list.append(utils.element2msg(self._wm.getElement(e.id)))
                self._publishChange(msg.author, "update", elements=e_list)
            elif msg.action == msg.REMOVE:
                for e in msg.elements:
                    to_ret.ids.append(self._wm.removeElement(utils.msg2element(e), msg.author))
                self._publishChange(msg.author, "remove", elements=msg.elements)
            elif msg.action == msg.REMOVE_RECURSIVE:
                for e in msg.elements:
                    to_ret.ids += self._wm.removeRecursive(utils.msg2element(e), msg.author, msg.relation_filter, msg.type_filter)
                self._publishChange(msg.author, "remove_recursive", elements=[utils.element2msg(Element(eid=cid)) for cid in to_ret.ids])
        if self._verbose:
            log.info("[WmModify]", "{} {} {}. Time: {:0.3f} secs".format(msg.author, msg.action, to_ret.ids, self._times.getLast()))
        return to_ret

    def run(self):
        rospy.spin()
