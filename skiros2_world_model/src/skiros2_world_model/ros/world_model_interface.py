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

from ontology_interface import *
import skiros2_common.ros.utils as utils
import skiros2_common.core.params as params
import skiros2_common.tools.logger as log
from skiros2_world_model.core.world_model_abstract_interface import WorldModelAbstractInterface
import copy
import numpy as np

class WorldModelInterface(OntologyInterface, WorldModelAbstractInterface):
    """
    Interface for scene services on a world model node
    """
    def __init__(self, author_name="test", monitor_callback=None):
        OntologyInterface.__init__(self, author_name)
        self._load_and_save = rospy.ServiceProxy('/skiros_wm/scene/load_and_save', srvs.WmLoadAndSave)
        self._set_relations = rospy.ServiceProxy('/skiros_wm/scene/set_relation', srvs.WmSetRelation)
        self._get = rospy.ServiceProxy('/skiros_wm/scene/get', srvs.WmGet)
        self._modify = rospy.ServiceProxy('/skiros_wm/scene/modify', srvs.WmModify)
        self._query_relations = rospy.ServiceProxy('/skiros_wm/scene/query_relations', srvs.WmQueryRelations)
        if monitor_callback:
            self._monitor = rospy.Subscriber("/skiros_wm/monitor", msgs.WmMonitor, monitor_callback)
        
    def getScene(self):
        """
        Return all elements in the scene
        """
        return self.getBranch("skiros:Scene-0")
        
    def setMonitorCallback(self, callback):
        self._monitor = rospy.Subscriber("/skiros_wm/monitor", msgs.WmMonitor, callback)
        
    def load(self, filename):
        msg = srvs.WmLoadAndSaveRequest()
        msg.action = msg.LOAD
        msg.filename = filename
        res = self._call(self._load_and_save, msg)
        if(res):
            return res.ok
        return False
        
    def save(self, filename):
        msg = srvs.WmLoadAndSaveRequest()
        msg.action = msg.SAVE
        msg.filename = filename
        res = self._call(self._load_and_save, msg)
        if(res):
            return res.ok
        return False
        
    def setRelation(self, subj, pred, obj, value=True):
        msg = srvs.WmSetRelationRequest()
        msg.author = self._author_name
        msg.relation = utils.relation2msg(utils.makeRelation(subj, pred, obj))
        msg.value = value
        res = self._call(self._set_relations, msg)
        if(res):
            return res.ok
        return False

    def _resolveLocalRelations(self, e):    
        for r in e._local_relations:
            sub_e = r['dst']
            sub_e.addRelation(e._id, r['type'], "-1")
            if sub_e._id=="":
                if self.addElement(sub_e)<0:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to add local element {}".format(sub_e))
            else:
                if self.updateElement(sub_e)<0:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to update local element {}".format(sub_e))
        e._local_relations = list()
    
    def addElement(self, e):
        msg = srvs.WmModifyRequest()
        msg.author = self._author_name
        msg.elements.append(utils.element2msg(e))
        msg.action = msg.ADD
        res = self._call(self._modify, msg)
        if(res):
            e._id = res.ids[0]
            self._resolveLocalRelations(e)
            return e._id
        return -1

    def updateElement(self, e):
        msg = srvs.WmModifyRequest()
        msg.author = self._author_name
        msg.elements.append(utils.element2msg(e))
        msg.action = msg.UPDATE
        res = self._call(self._modify, msg)
        if(res):
            return e._id
        return -1
        
    def removeElement(self, e, recursive=True, rel_filter=":sceneProperty", type_filter=""):
        #print "{}".format(eid)
        msg = srvs.WmModifyRequest()
        msg.author = self._author_name
        msg.type_filter = type_filter
        msg.relation_filter = rel_filter
        msg.elements.append(utils.element2msg(e))
        if recursive:
            msg.action = msg.REMOVE_RECURSIVE
        else:
            msg.action = msg.REMOVE
        res = self._call(self._modify, msg)
        if(res):
            return 1
        return -1

    def resolveElements(self, e):
        msg = srvs.WmGetRequest()
        msg.element = utils.element2msg(e)
        msg.action = msg.RESOLVE
        res = self._call(self._get, msg)
        if(res):
            return [utils.msg2element(x) for x in res.elements]
            
    def resolveElement(self, e):
        #TODO: change this to return only first element in list
        msg = srvs.WmGetRequest()
        msg.element = utils.element2msg(e)
        msg.action = msg.RESOLVE
        res = self._call(self._get, msg)
        if(res):
            return [utils.msg2element(x) for x in res.elements]

    def instanciate(self, uri, recursive=False, relations=list(), relation_filter="skiros:hasA", antiloop_bind=dict()):
        """
        Gets a template individual, adds it to the scene and returns a corresponding element
        
        If recursive, instanciate all individuals related to the starting individual
        """
        template = self.getTemplateElement(uri)
        relcopy = copy.deepcopy(template._relations)
        template._relations = relations
        self.addElement(template)
        if recursive:
            antiloop_bind[template._label] = template._id
            for r in relcopy:
                if (r['type']==relation_filter or relation_filter=="") and r['src']=="-1":
                    if r['dst'] in antiloop_bind:
                        rcopy = copy.deepcopy(r)
                        rcopy['src'] = template._id
                        rcopy['dst'] = antiloop_bind[r['dst']]
                        self.setRelations([rcopy])
                    else:
                        rcopy = copy.deepcopy(r)
                        rcopy['src'] = template._id
                        rcopy['dst'] = "-1"
                        r['dst'] = self.instanciate(r['dst'], True, [rcopy], relation_filter, antiloop_bind)._id
                        template._relations.append(r)
        return template
        
    def getTemplateElement(self, uri):
        msg = srvs.WmGetRequest()
        e = msgs.WmElement()
        e.label = uri
        msg.element = e
        msg.action = msg.GET_TEMPLATE
        res = self._call(self._get, msg)
        if(res):
            return utils.msg2element(res.elements[0])

    def getElement(self, eid):
        msg = srvs.WmGetRequest()
        e = msgs.WmElement()
        e.id = eid
        msg.element = e
        msg.action = msg.GET
        res = self._call(self._get, msg)
        if(res):
            return utils.msg2element(res.elements[0])
                    
    def getBranch(self, eid, relation_filter=":sceneProperty", type_filter=""):
        """
        Get all elements related to the starting one. Answer can be filtered
        """
        msg = srvs.WmGetRequest()
        e = msgs.WmElement()
        e.id = eid
        msg.element = e
        msg.action = msg.GET_RECURSIVE
        msg.relation_filter = relation_filter
        msg.type_filter = type_filter
        res = self._call(self._get, msg)
        if(res):
            return [utils.msg2element(x) for x in res.elements]

    def _getReasonersRel(self, subj, pred, obj):
        if pred!="" and subj!="" and obj!="" and subj!="-1" and obj!="-1":
            try:
                s = self.getElement(subj)
                o = self.getElement(obj)
                reasoner = s._getReasoner(pred)
                if pred in reasoner.computeRelations(s, o):
                   return [{"src": subj, "type": pred, "dst": obj}]
            except KeyError:
                pass
        return list()
                
    def getRelations(self, subj, pred, obj):
        msg = srvs.WmQueryRelationsRequest()
        msg.relation = utils.relation2msg(utils.makeRelation(subj, pred, obj))
        res = self._call(self._query_relations, msg)
        if(res):
            return [utils.msg2relation(x) for x in res.matches] + self._getReasonersRel(subj, pred, obj)

    def removeRelations(self, relations):
        msg = srvs.WmModifyRequest()
        msg.author = self._author_name
        msg.relations = relations
        msg.action = msg.REMOVE
        res = self._call(self._modify, msg)
        if(res):
            return res.return_code
            
    def resolveElements2(self, keys, ph):
        """
        Return all elements matching the profile in input (type, label, properties and relations)

        Keys: a key list pointing out the params to be resolved
        ph: a ParamHandler class     
        
        """
        first = {}
        couples = {}
        print_out = False
        for key in keys:
            first[key] = np.array(self.resolveElement(ph.getParamValue(key)))
            if not first[key].any():
                log.warn("resolveElements", "No input found for param {}. Resolving: {}".format(key, ph.getParamValue(key).printState(True)))
        all_keys = [key for key, _ in ph._params.iteritems()]
        coupled_keys = []
        overlap_keys = []
        relations_done = set([])
        #Build tuples of concording parameters
        for i in range(len(all_keys)):#Loop over all keys
            key_base = all_keys[i]
            if not isinstance(ph.getParamValue(key_base), Element): continue
            for j in ph.getParamValue(key_base)._relations:#Loop over relation constraints
                #print j
                if j["src"]=="-1":#-1 is the special autoreferencial value
                    key2 = j["dst"]  
                    key = key_base
                    rel_id = key_base+j["type"]+j["dst"]
                    if rel_id in relations_done:#Skip relation with previous indexes, already considered
                        continue
                    else:
                        #print rel_id
                        relations_done.add(rel_id)
                else:
                    key2 = key_base
                    key = j["src"]     
                    rel_id = j["src"]+j["type"]+key_base
                    if rel_id in relations_done:#Skip relation with previous indexes, already considered
                        continue
                    else:
                        #print rel_id
                        relations_done.add(rel_id)
                if not ph.hasParam(key) or not ph.hasParam(key2): #Check necessary because at the moment ._relations contains a mix Toclean
                    continue
                this = ph.getParamValue(key)
                other = ph.getParamValue(key2)
                #print "{} {}".format(key, key2)
                if this.getIdNumber()>=0 and other.getIdNumber()>=0:#If both parameters are already set, no need to resolve..
                    continue
                if this.getIdNumber()>=0: set1 = [this]
                else: 
                    if ph.getParam(key).paramType()==params.ParamTypes.Optional: continue 
                    else: set1 = first[key]
                if other.getIdNumber()>=0: set2 = [other]
                else:
                    if ph.getParam(key2).paramType()==params.ParamTypes.Optional: continue 
                    else: set2 = first[key2]
                if (key, key2) in couples:
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if bool(self.getRelations(e1._id, j["type"], e2._id)) == j['state']]
                    if temp:
                        couples[(key, key2)] = np.concatenate(couples[(key, key2)], np.array(temp))
                    else:
                        log.warn("resolveElements", "No input for params {} {}. Resolving: {} {}".format(key, key2, ph.getParamValue(key).printState(True), ph.getParamValue(key2).printState(True)))
                else:
                    if key in coupled_keys: overlap_keys.append(key)
                    else: coupled_keys.append(key)
                    if key2 in coupled_keys: overlap_keys.append(key2)
                    else: coupled_keys.append(key2)
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if bool(self.getRelations(e1._id, j["type"], e2._id)) == j['state']]
                    couples[(key, key2)] = np.array(temp)
                    if not temp:
                        log.warn("resolveElements", "No input for params {} {}. Resolving: {} {}".format(key, key2, ph.getParamValue(key).printState(True), ph.getParamValue(key2).printState(True)))
        #Merge the tuples with an overlapping key
        if overlap_keys:
            loop = True
            iters = 5
            while loop:#Iterate until no shared keys are found
                iters-=1
                if iters==0:
                    raise
                loop = False
                coupled_keys2 = []
                merged = {}
                #print 'qui:'         
                for k1, s1 in couples.iteritems():
                    for k2, s2 in couples.iteritems():
                        shared_k = [k for k in k1 if k in k2]
                        if k1==k2 or not shared_k:
                            continue
                        loop = True
                        skip = True
                        for i in k1:
                            if not i in coupled_keys2:
                                coupled_keys2.append(i)
                                skip=False
                        for i in k2:
                            if not i in coupled_keys2:
                                coupled_keys2.append(i)
                                skip=False
                        if skip: continue#If it was already considered, skip
                        rk, rs = self._intersect(k1,k2,s1,s2, shared_k)
                        merged[rk] = rs#Temporary store merged tuple
                for key in keys:#Add not merged tuples
                    if not key in coupled_keys2:
                        for k1, s1 in couples.iteritems():
                            if key in k1:
                                merged[k1] = s1 
                couples = merged   
        #Add back keys that are not coupled to others
        for key in keys:
            if not key in coupled_keys:
                couples[key] = first[key]
        if print_out:
            for k, v in couples.iteritems():
                s = "{}:".format(k)
                for i in v:
                    if not isinstance(i, Element):
                        s += "["
                        for j in i:
                            s += "{},".format(j)
                        s += "]"
                    else:                        
                        s += "{},".format(i)
                print s
        return couples
               
    def _concatenate(self, a, b):
        if not isinstance(a, np.ndarray):
            a = np.array([a])
        if not isinstance(b, np.ndarray):
            b = np.array([b])
        return np.concatenate((a,b))
       
    def _intersect(self, k1, k2, s1, s2, shared_k):
        a = [k1.index(k) for k in shared_k]
        b = [k2.index(k) for k in shared_k]
        c = np.arange(len(k1))
        d = np.arange(len(k2))
        d = np.delete(d, b)
        keys = []
        #Remove constant sets
        for k in k1:
            keys.append(k)
        for k in k2:
            if not k in shared_k:
                keys.append(k)
        #print keys
        sets = []
        #print c
        #print d
        for v1 in s1:
            for v2 in s2:
                append=True
                for i in range(len(shared_k)):
                    #print str(v1[a[i]].printState()) + 'vs' + str(v1[b[i]].printState()) + '=' + str(v1[a[i]]!=v2[b[i]])
                    if v1[a[i]]!=v2[b[i]]:
                        append=False
                if append:
                    sets.append(np.array(self._concatenate(v1[c], v2[d])))
        return tuple(keys), np.array(sets)
