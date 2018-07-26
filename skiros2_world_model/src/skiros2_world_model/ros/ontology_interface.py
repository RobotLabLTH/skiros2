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
import skiros2_msgs.srv as srvs
import skiros2_msgs.msg as msgs
import skiros2_common.ros.utils as utils
import skiros2_common.tools.logger as log
from std_srvs.srv import SetBool, SetBoolRequest
from skiros2_common.core.world_element import *
from skiros2_world_model.core.world_model_abstract_interface import OntologyAbstractInterface

class OntologyInterface(OntologyAbstractInterface):
    """
    Interface for ontology services on a world model node
    """
    def __init__(self, author_name="test"):
        self._author_name = author_name
        self._lock = rospy.ServiceProxy('wm/lock', SetBool)
        self._ontology_query = rospy.ServiceProxy('wm/ontology/query', srvs.WoQuery)
        self._ontology_modify = rospy.ServiceProxy('wm/ontology/modify', srvs.WoModify)
        self._load_and_save = rospy.ServiceProxy('wm/ontology/load_and_save', srvs.WoLoadAndSave)
        log.info("[{}] ".format(self.__class__.__name__), "Waiting wm communications...")
        self._ontology_modify.wait_for_service()
        log.info("[{}] ".format(self.__class__.__name__), "Wm communications active.")
        self._def_prefix = ":"
        self._sub_classes_cache = {}
        self._sub_properties_cache = {}

    def isConnected(self):
        try:
            rospy.wait_for_service(self._ontology_query.resolved_name, 0.0)
            return True
        except rospy.ROSException, e:
            return False

    def lock(self):
        """
        @brief Lock the ontology server mutex to signal exclusive access

        Note: locking is NOT mandatory and all functions will still work without locking the server
        """
        return self._call(self._lock, SetBoolRequest(True)).success

    def unlock(self):
        """
        @brief Unlock the ontology server mutex
        """
        return self._call(self._lock, SetBoolRequest(False)).success


    def addClass(self, class_uri, parent_uri, context):
        #print query
        req = srvs.WoModifyRequest()
        req.context = context
        rmsg = msgs.Statement()
        rmsg.value = True
        rmsg.relation = utils.makeRelationMsg(class_uri, "rdf:type", "owl:class")
        req.statements.append(rmsg)
        rmsg = msgs.Statement()
        rmsg.value = True
        rmsg.relation = utils.makeRelationMsg(class_uri, "rdfs:subClassOf", parent_uri)
        req.statements.append(rmsg)
        res = self._call(self._ontology_modify, req)
        if res:
            if res.ok:
                return True
        return False

    def add_datatype(self, data_uri):
        pass

    def add_individual(self, element, context):
        """
        @brief Store an individual in a specified ontology
        """
        #TODO:
        pass

    def add_context_graph(self, context_id):
        """
        @brief Creates a new subgraph

        @param context_id the id of the graph
        """

    def load(self, filename, context='scene'):
        """
        @brief Load a file in a context
        """
        msg = srvs.WoLoadAndSaveRequest()
        msg.action = msg.LOAD
        msg.filename = filename
        msg.context = context
        res = self._call(self._load_and_save, msg)
        if(res):
            return res.ok
        return False

    def save(self, filename, context='scene'):
        """
        @brief Save a context in a file
        """
        msg = srvs.WoLoadAndSaveRequest()
        msg.action = msg.SAVE
        msg.filename = filename
        msg.context = context
        res = self._call(self._load_and_save, msg)
        if(res):
            return res.ok
        return False

    def queryOntology(self, query, cut_prefix=True, context=""):
        """
        @brief Direct SPARQL interface. query should be a string in SPARQL syntax

        @param cut_prefix If True the prefix in returned values is removed
        @param context The context in which executing the query

        @return a list of strings
        """
        req = srvs.WoQueryRequest()
        req.query_string = query
        req.context = context
        req.cut_prefix = cut_prefix
        return self._call(self._ontology_query, req).answer

    def setDefaultPrefix(self, default_prefix):
        self._def_prefix = default_prefix

    def addPrefix(self, uri):
        """
        @brief Formats the uri for a SPARQL query

        Adds prefix to uri or does nothing if uri has already the prefix

        Adds < > brakets, if necessary
        """
        if uri.find("#") is -1 and uri.find(":") is -1:
            uri = self._def_prefix + ":" + uri
        elif not uri.find("#") is -1 and uri.find("<") is -1:
            uri = "<" + uri + ">"
        elif not uri.find(":") is -1 and uri.find(":") is 0:
            uri = self._def_prefix + uri
        return uri

    def removePrefix(self, parent_class):
        if parent_class.find("#")>=0:
            return parent_class.split("#")[-1]
        elif parent_class.find(":")>=0:
            return parent_class.split(":")[-1]
        else:
            return parent_class

    def getIndividuals(self, parent_class, recursive=True):
        """
        @brief Return a list of all individuals of the type of the parent_class
        @param parent_class The class of individuals
        @param recursive If recursive, returns also individuals of subclasses
        """
        if recursive:
            return self.queryOntology("SELECT ?x WHERE { ?x rdf:type/rdfs:subClassOf* " + self.addPrefix(parent_class) + " . } ")
        else:
            return self.queryOntology("SELECT ?x where {?x rdf:type+ "+self.addPrefix(parent_class)+"}")

    def getType(self, uri):
        return self.queryOntology("SELECT ?x where {"+self.addPrefix(uri)+" rdf:type ?x}")

    def getTriples(self, subj=None, pred=None, obj=None):
        """
        @brief Return matching triples.

        Note: at least one between subj, pred or obj must left blank for this function to work.
        """
        if subj: subj = self.addPrefix(subj)
        else: subj = "?x"
        if pred: pred = self.addPrefix(pred)
        else: pred = "?y"
        if obj:  obj = self.addPrefix(obj)
        else:  obj = "?z"
        return self.queryOntology("SELECT * WHERE { "+ "{} {} {}".format(subj, pred, obj)+" . } ")

    def getSuperClass(self, child_class):
        """
        @brief Return the parent class of child_class
        """
        to_ret = self.queryOntology("SELECT ?x WHERE { "+ self.addPrefix(child_class) +" rdfs:subClassOf ?x. } ")
        if not to_ret:
            log.error("[getSuperClass]", "No super class found for {}".format(child_class))
        return to_ret[0]

    def get_sub_classes(self, parent_class, recursive=True):
        """
        @brief Return the child classes of parent_class. If recursive=True, returns also sub childs classes
        """
        if(self._sub_classes_cache.has_key(parent_class)):
            return self._sub_classes_cache[parent_class]
        if recursive:
            to_ret = self.queryOntology("SELECT ?x WHERE { ?x rdfs:subClassOf* " + self.addPrefix(parent_class) + " . } ")
        else:
            to_ret = self.queryOntology("SELECT ?x WHERE { ?x rdfs:subClassOf " + self.addPrefix(parent_class) + " . } ")
        self._sub_classes_cache[parent_class] = to_ret
        return to_ret

    def get_sub_properties(self, parent_property, recursive=True):
        """
        @brief Return the child properties of parent_property. If recursive=True, returns also sub childs properties
        """
        if(self._sub_properties_cache.has_key(parent_property)):
            return self._sub_properties_cache[parent_property]
        if recursive:
            to_ret = self.queryOntology("SELECT ?x WHERE { ?x rdfs:subPropertyOf* " + self.addPrefix(parent_property) + " . } ")
        else:
            to_ret = self.queryOntology("SELECT ?x WHERE { ?x rdfs:subPropertyOf " + self.addPrefix(parent_property) + " . } ")
        self._sub_properties_cache[parent_property] = to_ret
        return to_ret

    def _call(self, service, msg):
        try:
            resp1 = service(msg)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            raise Exception("Service call failed: %s"%e)