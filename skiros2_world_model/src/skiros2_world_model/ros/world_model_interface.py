from ontology_interface import *
import skiros2_common.ros.utils as utils
import skiros2_common.core.params as params
import skiros2_common.tools.logger as log
from skiros2_world_model.core.world_model_abstract_interface import WorldModelAbstractInterface
import copy
import numpy as np
from inspect import getframeinfo, stack



class WorldModelInterface(OntologyInterface, WorldModelAbstractInterface):
    """
    Interface for scene services on a world model node
    """
    _elements_cache = {}

    def __init__(self, author_name="test", make_cache=False):
        OntologyInterface.__init__(self, author_name)
        self._set_relations = rospy.ServiceProxy('wm/scene/set_relation', srvs.WmSetRelation)
        self._get = rospy.ServiceProxy('wm/get', srvs.WmGet)
        self._modify = rospy.ServiceProxy('wm/modify', srvs.WmModify)
        self._query_relations = rospy.ServiceProxy('wm/scene/query_relations', srvs.WmQueryRelations)
        self._last_snapshot_id = ""
        self._make_cache = make_cache
        self._external_monitor_cb = None
        self._monitor = rospy.Subscriber("wm/monitor", msgs.WmMonitor, self._monitor_cb, queue_size=100)
    
    def _debug_info(self):
        """
        @brief Retrives the function caller file and line
        """
        i=2
        caller = getframeinfo(stack()[i][0])
        while "world_model_interface" in caller.filename:
            i+=1
            caller = getframeinfo(stack()[i][0])
        return "%s:%d" % (caller.filename[caller.filename.rfind("/"):], caller.lineno)
        
    def _monitor_cb(self, msg):
        if self._make_cache:
            if self._last_snapshot_id != msg.prev_snapshot_id or msg.action == 'reset':
                WorldModelInterface._elements_cache.clear()
            self._last_snapshot_id = msg.snapshot_id
            for elem in msg.elements:
                elem = utils.msg2element(elem)
                if msg.action == 'update' or msg.action == 'update_properties' or msg.action == 'add':
                    WorldModelInterface._elements_cache[elem.id] = elem
                elif msg.action == 'remove' or msg.action == 'remove_recursive':
                    if elem.id in WorldModelInterface._elements_cache:
                        del WorldModelInterface._elements_cache[elem.id]
                else:
                    log.error("[WmMonitor]", "Command {} not recognized.".format(msg.action))
            if msg.relation:
                rel = utils.msg2relation(msg.relation[0])
                if rel['src'] in WorldModelInterface._elements_cache:
                    del WorldModelInterface._elements_cache[rel['src']]
                if rel['dst'] in WorldModelInterface._elements_cache:
                    del WorldModelInterface._elements_cache[rel['dst']]
        if self._external_monitor_cb:
            self._external_monitor_cb(msg)

    def get_scene_name(self):
        """
        @brief Return the name of the scene
        """
        return rospy.get_param('wm/init_scene', "")

    def get_scene(self):
        """
        @brief Returns a triple with all elements in the scene and the associated uuid
        """
        msg = srvs.WmGetRequest()
        e = msgs.WmElement()
        e.id = "skiros:Scene-0"
        msg.context = 'scene'
        msg.element = e
        msg.action = msg.GET_RECURSIVE
        msg.relation_filter = "skiros:sceneProperty"
        msg.type_filter = ""
        res = self._call(self._get, msg)
        if(res):
            return ([utils.msg2element(x) for x in res.elements], res.snapshot_id)

    def set_monitor_cb(self, cb):
        self._external_monitor_cb = cb

    def set_relation(self, subj, pred, obj, value=True):
        msg = srvs.WmSetRelationRequest()
        msg.author = self._author_name + self._debug_info()
        msg.relation = utils.relation2msg(utils.makeRelation(subj, pred, obj))
        msg.value = value
        res = self._call(self._set_relations, msg)
        if(res):
            return res.ok
        return False

    def _resolve_local_relations(self, e):
        for r in e._local_relations:
            sub_e = r['dst']
            sub_e.addRelation(e._id, r['type'], "-1")
            if sub_e._id == "":
                if self.add_element(sub_e) < 0:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to add local element {}".format(sub_e))
            else:
                if self.update_element(sub_e) < 0:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to update local element {}".format(sub_e))
        e._local_relations = list()

    def add_elements(self, es, context_id='scene'):
        msg = srvs.WmModifyRequest()
        msg.context = context_id
        msg.author = self._author_name + self._debug_info()
        for e in es:
            msg.elements.append(utils.element2msg(e))
        msg.action = msg.ADD
        res = self._call(self._modify, msg)
        to_ret = list()
        if res:
            for old, new in zip(es, res.elements):
                to_ret.append(utils.msg2element(new))
                old._id = new.id
                self._resolve_local_relations(old)
        return to_ret

    def add_element(self, e, context_id='scene'):
        """
        @brief Add an element to the scene. The id is updated with the one assigned from the world model
        """
        res = self.add_elements([e], context_id)
        if not res:
            return None
        else:
            return res[0]

    def update_element(self, e, context_id='scene'):
        """
        @brief Update properties and relations of an element
        """
        msg = srvs.WmModifyRequest()
        msg.context = context_id
        msg.author = self._author_name + self._debug_info()
        msg.elements.append(utils.element2msg(e))
        msg.action = msg.UPDATE
        res = self._call(self._modify, msg)
        if res:
            return e.id
        return -1

    def update_element_properties(self, e, reasoner="", context_id='scene'):
        """
        @brief Update the properties of an element (ignores the relations)
        """
        msg = srvs.WmModifyRequest()
        msg.context = context_id
        msg.author = self._author_name + self._debug_info()
        msg.elements.append(utils.element2msg(e))
        msg.action = msg.UPDATE_PROPERTIES
        msg.type_filter = reasoner
        res = self._call(self._modify, msg)
        if(res):
            return e.id
        return -1

    def remove_element(self, e, recursive=True, rel_filter=":sceneProperty", type_filter="", context_id='scene'):
        """
        @brief Removes an element from the scene. e can be an Element or an id
        """
        #print "{}".format(eid)
        msg = srvs.WmModifyRequest()
        msg.context = context_id
        msg.author = self._author_name + self._debug_info()
        msg.type_filter = type_filter
        msg.relation_filter = rel_filter
        if isinstance(e, Element):
            msg.elements.append(utils.element2msg(e))
        elif isinstance(e, str):
            e = Element("", "", e)
            msg.elements.append(utils.element2msg(e))
        if recursive:
            msg.action = msg.REMOVE_RECURSIVE
        else:
            msg.action = msg.REMOVE
        res = self._call(self._modify, msg)
        if(res):
            e._id = ""
            return 1
        return -1

    def resolve_elements(self, e, context_id='scene'):
        msg = srvs.WmGetRequest()
        msg.context = context_id
        msg.element = utils.element2msg(e)
        msg.action = msg.RESOLVE
        res = self._call(self._get, msg)
        if(res):
            return [utils.msg2element(x) for x in res.elements]

    def resolve_element(self, e, context_id='scene'):
        res = self.resolve_elements(e, context_id)
        if res:
            return res[0]

    def instanciate(self, uri, recursive=False, relations=list(), relation_filter=["skiros:hasA", "skiros:contain"], antiloop_bind=set(), context_id='scene'):
        """
        Gets a template individual, adds it to the scene and returns a corresponding element

        If recursive, instanciate all individuals related to the starting individual
        """
        if isinstance(uri, basestring):
            template = self.get_template_element(uri)
        else:
            template = uri
        relcopy = copy.deepcopy(template._relations)
        template._relations = relations
        template = self.add_element(template, context_id=context_id)
        if recursive:
            antiloop_bind.add(template.id)
            for r in relcopy:
                if (r['type'] in relation_filter or not relation_filter) and r['src'] == "-1":
                    if not r['dst'] in antiloop_bind:
                        rcopy = copy.deepcopy(r)
                        rcopy['src'] = template.id
                        rcopy['dst'] = "-1"
                        r['dst'] = self.instanciate(r['dst'], True, [rcopy], relation_filter, antiloop_bind, context_id).id
                        template._relations.append(r)
        return template

    def get_template_element(self, uri, context_id='scene'):
        msg = srvs.WmGetRequest()
        e = msgs.WmElement()
        e.label = uri
        msg.context = context_id
        msg.element = e
        msg.action = msg.GET_TEMPLATE
        res = self._call(self._get, msg)
        if(res):
            return utils.msg2element(res.elements[0])

    def get_element(self, eid, context_id='scene'):
        if eid not in WorldModelInterface._elements_cache:
            msg = srvs.WmGetRequest()
            e = msgs.WmElement()
            e.id = eid
            msg.context = context_id
            msg.element = e
            msg.action = msg.GET
            res = self._call(self._get, msg)
            if not res:
                return None
            if self._make_cache:
                WorldModelInterface._elements_cache[eid] = utils.msg2element(res.elements[0])
            else:
                return utils.msg2element(res.elements[0])
        return WorldModelInterface._elements_cache[eid]

    def get_branch(self, eid, relation_filter=":sceneProperty", type_filter="", context_id='scene'):
        """
        Get all elements related to the starting one. Answer can be filtered
        """
        msg = srvs.WmGetRequest()
        e = msgs.WmElement()
        e.id = eid
        msg.context = context_id
        msg.element = e
        msg.action = msg.GET_RECURSIVE
        msg.relation_filter = relation_filter
        msg.type_filter = type_filter
        res = self._call(self._get, msg)
        if(res):
            return [utils.msg2element(x) for x in res.elements]

    def get_reasoner_relations(self, subj, pred, obj):
        reasoner = self.get_reasoner(pred)
        if reasoner:
            if pred in reasoner.computeRelations(subj, obj):
                return [{"src": subj.id, "type": pred, "dst": obj.id}]
            else:
                return list()
    
    def get_reasoner(self, pred):
        """
        @brief Returns the reasoner associated with the predicate, or None
        """
        try:
            return Element("")._getReasoner(pred)
        except KeyError:
            #No reasoner associated with the relation
            return None

    def get_relations(self, subj, pred, obj):
        if pred != "" and subj != "" and obj != "" and subj != "-1" and obj != "-1":
            subj = self.get_element(subj)
            obj = self.get_element(obj)
            rels = self.get_reasoner_relations(subj, pred, obj)
            if rels is not None:
                return rels
            else:
                return subj.getRelations(pred=self.get_sub_properties(pred), obj=obj.id)
        msg = srvs.WmQueryRelationsRequest()
        msg.relation = utils.relation2msg(utils.makeRelation(subj, pred, obj))
        res = self._call(self._query_relations, msg)
        if(res):
            return [utils.msg2relation(x) for x in res.matches]

    def check_relation(self, subj, pred, obj, state, abstract):
        if abstract:
            if subj.hasProperty("skiros:Template"):
                e1 = subj.getProperty("skiros:Template").value
            else:
                e1 = subj.id
            if obj.hasProperty("skiros:Template"):
                e2 = obj.getProperty("skiros:Template").value
            else:
                e2 = obj.id
            #print "CHECKING {} {} {} = {}".format(e1, pred, e2, (e2 in self.getTriples(e1, pred)) == state)
            return (e2 in self.get_triples(e1, pred)) == state
        else:
            return bool(self.get_relations(subj.id, pred, obj.id)) == state

    def resolve_elements2(self, keys, ph, verbose=False):
        """
        Return all elements matching the profile in input (type, label, properties and relations)

        Keys: a key list pointing out the params to be resolved
        ph: a ParamHandler class

        """
        first = {}
        couples = {}
        for key in keys:
            first[key] = np.array(self.resolve_elements(ph.getParamValue(key)))
            if not first[key].any():
                log.warn("resolve_elements", "No input found for param {}. Resolving: {}".format(key, ph.getParamValue(key).printState(True)))
        all_keys = ph.keys()
        coupled_keys = []
        overlap_keys = []
        relations_done = set([])
        # Build tuples of concording parameters
        for i in range(len(all_keys)):  # Loop over all keys
            key_base = all_keys[i]
            if not isinstance(ph.getParamValue(key_base), Element):
                continue
            for j in ph.getParamValue(key_base)._relations:  # Loop over relation constraints
                #print j
                if j["src"] == "-1":  # -1 is the special autoreferencial value
                    key2 = j["dst"]
                    key = key_base
                    rel_id = key_base + j["type"] + j["dst"]
                    if rel_id in relations_done:  # Skip relation with previous indexes, already considered
                        continue
                    else:
                        #print rel_id
                        relations_done.add(rel_id)
                else:
                    key2 = key_base
                    key = j["src"]
                    rel_id = j["src"] + j["type"] + key_base
                    if rel_id in relations_done:  # Skip relation with previous indexes, already considered
                        continue
                    else:
                        #print rel_id
                        relations_done.add(rel_id)
                if not ph.hasParam(key) or not ph.hasParam(key2):  # Check necessary because at the moment ._relations contains a mix Toclean
                    continue
                this = ph.getParamValue(key)
                other = ph.getParamValue(key2)
                if this.getIdNumber() >= 0 and other.getIdNumber() >= 0:  # If both parameters are already set, no need to resolve..
                    continue
                if this.getIdNumber() >= 0:
                    set1 = [this]
                else:
                    if ph.getParam(key).paramType == params.ParamTypes.Optional:
                        abstract = ph.getParam(key).value
                        other = ph.getParam(key2).value
                        abstract._id = abstract.label
                        set1 = [abstract]
                    else:
                        set1 = first[key]
                if other.getIdNumber() >= 0:
                    set2 = [other]
                else:
                    if ph.getParam(key2).paramType == params.ParamTypes.Optional:
                        abstract = ph.getParam(key2).value
                        other = ph.getParam(key).value
                        abstract._id = abstract.label
                        set2 = [abstract]
                    else:
                        set2 = first[key2]
                if (key, key2) in couples:
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if self.check_relation(e1, j["type"], e2, j['state'], j['abstract'])]
                    if temp:
                        try:
                            couples[(key, key2)] = np.concatenate(couples[(key, key2)], np.array(temp))
                        except BaseException:
                            log.error("", "MERGING: {} and {} ".format(couples[(key, key2)], np.array(temp)))
                    else:
                        log.warn("resolve_elements", "No input for params {} {}. No match for: {} {} {}".format(key, key2, set1, j["type"], set2))
                else:
                    if key in coupled_keys:
                        overlap_keys.append(key)
                    else:
                        coupled_keys.append(key)
                    if key2 in coupled_keys:
                        overlap_keys.append(key2)
                    else:
                        coupled_keys.append(key2)
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if self.check_relation(e1, j["type"], e2, j['state'], j['abstract'])]
                    couples[(key, key2)] = np.array(temp)
                    if not temp:
                        log.warn("resolve_elements", "No input for params {} {}. No match for: {} {} {}".format(key, key2, set1, j["type"], set2))
        #Merge the tuples with an overlapping key
        if overlap_keys:
            loop = True
            iters = 5
            while loop:  # Iterate until no shared keys are found
                iters -= 1
                if iters == 0:
                    raise
                loop = False
                coupled_keys2 = []
                merged = {}
                #print 'qui:'
                for k1, s1 in couples.iteritems():
                    for k2, s2 in couples.iteritems():
                        shared_k = [k for k in k1 if k in k2]
                        if k1 == k2 or not shared_k:
                            continue
                        loop = True
                        skip = True
                        for i in k1:
                            if not i in coupled_keys2:
                                coupled_keys2.append(i)
                                skip = False
                        for i in k2:
                            if not i in coupled_keys2:
                                coupled_keys2.append(i)
                                skip = False
                        if skip:
                            continue  # If it was already considered, skip
                        rk, rs = self._intersect(k1, k2, s1, s2, shared_k)
                        merged[rk] = rs  # Temporary store merged tuple
                for key in keys:  # Add not merged tuples
                    if not key in coupled_keys2:
                        for k1, s1 in couples.iteritems():
                            if key in k1:
                                merged[k1] = s1
                couples = merged
        # Add back keys that are not coupled to others
        for key in keys:
            if not key in coupled_keys:
                couples[key] = first[key]
        if verbose:
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
        return np.concatenate((a, b))

    def _intersect(self, k1, k2, s1, s2, shared_k):
        a = [k1.index(k) for k in shared_k]
        b = [k2.index(k) for k in shared_k]
        c = np.arange(len(k1))
        d = np.arange(len(k2))
        d = np.delete(d, b)
        keys = []
        # Remove constant sets
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
                append = True
                for i in range(len(shared_k)):
                    #print str(v1[a[i]].printState()) + 'vs' + str(v1[b[i]].printState()) + '=' + str(v1[a[i]]!=v2[b[i]])
                    if v1[a[i]] != v2[b[i]]:
                        append = False
                if append:
                    sets.append(np.array(self._concatenate(v1[c], v2[d])))
        return tuple(keys), np.array(sets)
