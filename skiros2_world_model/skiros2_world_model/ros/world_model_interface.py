from .ontology_interface import *
import skiros2_common.ros.utils as utils
import skiros2_common.core.params as params
import skiros2_common.tools.logger as log
from skiros2_world_model.core.world_model_abstract_interface import WorldModelAbstractInterface
import copy
import numpy as np
from inspect import getframeinfo, stack
try:
    basestring
except NameError:
    basestring = str


class WorldModelInterface(OntologyInterface, WorldModelAbstractInterface):
    _elements_cache = {}

    def __init__(self, node, author_name, make_cache=False, allow_spinning=True):
        """
        @brief      Interface to world model scene services

                    This class uses ROS services to access and edit the
                    world model scene on a server node

        @param      author_name  (string) Id used to track changes on
                                 the server
        @param      make_cache   If true, keeps a local cache to speed
                                 up world model access
        """
        OntologyInterface.__init__(self, node, author_name, allow_spinning)
        self._node.declare_parameter("wm/init_scene", "")
        self._set_relations = self._node.create_client(srvs.WmSetRelation, 'wm/scene/set_relation')
        self._get = self._node.create_client(srvs.WmGet, 'wm/get')
        self._modify = self._node.create_client(srvs.WmModify, 'wm/modify')
        self._query_relations = self._node.create_client(srvs.WmQueryRelations, 'wm/scene/query_relations')
        self._init_scene = self._node.create_client(srvs.WmInitScene, 'wm/init_scene')
        for s in [self._get, self._set_relations, self._modify, self._query_relations]:
            while not s.wait_for_service(timeout_sec=1.0):
                log.warn("[{}]".format(self.__class__.__name__), "Service {} not available, waiting again ...".format(s.srv_name))
        self._last_snapshot_id = ""
        self._make_cache = make_cache
        self._external_monitor_cb = None
        self._monitor = self._node.create_subscription(msgs.WmMonitor, "wm/monitor", self._monitor_cb, 100)

    def _debug_info(self):
        """
        @brief Retrives the function caller file and line
        """
        i = 2
        caller = getframeinfo(stack()[i][0])
        while "world_model_interface" in caller.filename:
            i += 1
            caller = getframeinfo(stack()[i][0])
        return "%s:%d" % (caller.filename[caller.filename.rfind("/"):], caller.lineno)

    def _monitor_cb(self, msg):
        """
        @brief      Callback updating the cache when a change on wm is detected
        """
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

    def _resolve_local_relations(self, e):
        for r in e._local_relations:
            sub_e = r['dst']
            sub_e.addRelation(e._id, r['type'], "-1")
            if sub_e._id == "":
                res = self.add_element(sub_e)
                if res is None:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to add local element {}".format(sub_e))
            else:
                if self.update_element(sub_e) == -1:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to update local element {}".format(sub_e))
        e._local_relations = list()

    def get_scene_name(self):
        """
        @brief Get the name of the scene from the world-model server
        """
        res = self._call(self._init_scene, srvs.WmInitScene.Request())
        return res.init_scene

    def get_scene(self):
        """
        @brief      Get all scene elements

        @return     tuple(list(Element), string) a tuple with all
                    elements in the scene and the scene instance uuid
        """
        msg = srvs.WmGet.Request()
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
        """
        @brief      Set an external monitor callback

        @param      cb    (function)
        """
        self._external_monitor_cb = cb

    def set_relation(self, subj, pred, obj, value=True):
        """
        @brief      Sets a relation.

        @param      subj   (string)The subj uri
        @param      pred   (string)The predicate uri
        @param      obj    (string)The object uri
        @param      value  (Bool) True set the relation, False removes
                           it

        @return     (bool) False on failure
        """
        msg = srvs.WmSetRelation.Request()
        msg.author = self._author_name + self._debug_info()
        msg.relation = utils.relation2msg(utils.makeRelation(subj, pred, obj))
        msg.value = value
        res = self._call(self._set_relations, msg)
        if(res):
            return res.ok
        return False

    def is_scene_element(self, eid):
        """
        @brief      Determines whether the specified eid is scene element.

        @param      eid   (string) The element id

        @return     True if the specified eid is a scene element, False otherwise.
        """
        return eid.find("-") > 0

    def add_elements(self, es, context_id='scene'):
        """
        @brief      Add elements to the scene. The id is updated with
                    the one assigned from the world model

        @param      es          list(Elements) without id
        @param      context_id  (string)Ontology context identifier

        @return     list(Elements) with id
        """
        msg = srvs.WmModify.Request()
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
        @brief      Like add_elements, but for one element

        @param      e           an element without id
        @param      context_id  (string)Ontology context identifier

        @return     (Element) with id
        """
        res = self.add_elements([e], context_id)
        if not res:
            return None
        else:
            return res[0]

    def update_element(self, e, context_id='scene'):
        """
        @brief      Update properties and relations of an element

        @param      e           (Element) to update
        @param      context_id  (string)Ontology context identifier

        @return     (int) id of the element, or -1 if an error occurs
        """
        msg = srvs.WmModify.Request()
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
        @brief      Update the properties of an element, ignoring the
                    relations. More efficient than full update

        @param      e           (Element) to update
        @param      reasoner    (string) If specified, only updates
                                properties related to the reasoner
        @param      context_id  (string)Ontology context identifier

        @return     (int) id of the element, or -1 if an error occurs
        """
        msg = srvs.WmModify.Request()
        msg.context = context_id
        msg.author = self._author_name + self._debug_info()
        msg.elements.append(utils.element2msg(e))
        msg.action = msg.UPDATE_PROPERTIES
        msg.type_filter = reasoner
        res = self._call(self._modify, msg)
        if(res):
            return e.id
        return -1

    def remove_element(self, e, recursive=True, rel_filter="skiros:sceneProperty", type_filter="", context_id='scene'):
        """
        @brief      Removes an element from the scene. e can be an
                    Element or an id

        @param      e            (Element/string) Element or id of element to remove
        @param      recursive    The recursive, if True, remove all children matching the rel_filter
        @param      rel_filter   The relative filter
        @param      type_filter  The type filter
        @param      context_id  (string)Ontology context identifier

        @return     (int) 1 on success, -1 if an error occurs
        """
        msg = srvs.WmModify.Request()
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
        """
        @brief      Find all elements matching the input *type, label
                    and properties)

        @param      e           An element to match
        @param      context_id  (string)Ontology context identifier

        @return     list(Element) List of matches
        """
        msg = srvs.WmGet.Request()
        msg.context = context_id
        msg.element = utils.element2msg(e)
        msg.action = msg.RESOLVE
        res = self._call(self._get, msg)
        if(res):
            return [utils.msg2element(x) for x in res.elements]

    def resolve_element(self, e, context_id='scene'):
        """
        @brief      Like resolve_elements, but returns only the first
                    match

        @param      e           An element to match
        @param      context_id  (string)Ontology context identifier

        @return     Element or None if no match is found
        """
        res = self.resolve_elements(e, context_id)
        if res:
            return res[0]

    def instanciate(self, uri, recursive=False, relations=list(), relation_filter=["skiros:hasA", "skiros:contain"], antiloop_bind=set(), context_id='scene'):
        """
        @brief      Gets a template individual, adds it to the scene and
                    returns a corresponding element

        @param      uri              (string)URI of the individual to
                                     instanciate
        @param      recursive        (bool)If true, instanciate all
                                     individuals related to the starting
                                     individual with releations in
                                     releation_filter
        @param      relations        The relations
        @param      relation_filter  list(string) Relations to consider
                                     for recursive instanciation
        @param      antiloop_bind    The antiloop bind
        @param      context_id       (string)Ontology context identifier

        @return     (Element)
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
                        r['dst'] = self.instanciate(r['dst'], True, [rcopy], relation_filter,
                                                    antiloop_bind, context_id).id
                        template._relations.append(r)
        return template

    def get_template_element(self, uri, context_id='scene'):
        """
        @brief      Gets a template element, an ontology individual that
                    is not instanciated in the world model.

        @param      uri         (string)URI of the individual
        @param      context_id  (string)Ontology context identifier

        @return     (Element)
        """
        msg = srvs.WmGet.Request()
        e = msgs.WmElement()
        e.label = uri
        msg.context = context_id
        msg.element = e
        msg.action = msg.GET_TEMPLATE
        res = self._call(self._get, msg)
        if(res):
            return utils.msg2element(res.elements[0])

    def get_element(self, eid, context_id='scene'):
        """
        @brief      Gets an element instanciated in the world model.

        @param      eid         (string)Id of the element instance
        @param      context_id  (string)Ontology context identifier

        @return     (Element)
        """
        if eid not in WorldModelInterface._elements_cache:
            msg = srvs.WmGet.Request()
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

    def get_branch(self, eid, relation_filter="skiros:sceneProperty", type_filter="", context_id='scene'):
        """
        @brief      Get an element and related children elements. Answer
                    can be filtered

        @param      eid              (string)Id of branch root element
        @param      relation_filter  (string)Relation type to consider
                                     to retrieve the children
        @param      type_filter      (string)Filter on Element class
                                     type (e.g. skiros:Manipulatable)
        @param      context_id       (string)Ontology context identifier

        @return     list(Element)
        """
        msg = srvs.WmGet.Request()
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
        """
        @brief      Use a reasoner to evaluate if a relation is true or
                    false

        @param      subj  (string) The relation subject
        @param      pred  (string) The relation predicate
        @param      obj   (string) The relation object

        @return     A list containing the relation or an empty list if
                    the relation is not found
        """
        reasoner = self.get_reasoner(pred)
        if reasoner:
            if pred in reasoner.computeRelations(subj, obj):
                return [{"src": subj.id, "type": pred, "dst": obj.id}]
            else:
                return list()

    def get_reasoner(self, pred):
        """
        @brief      Returns the reasoner associated with the predicate

        @param      pred  (string) The predicate

        @return     The associated reasoner instance, or None if not
                    found
        """
        try:
            return Element("")._getReasoner(pred)
        except KeyError:
            # No reasoner associated with the relation
            return None

    def get_relations(self, subj, pred, obj):
        """
        @brief      Get all relations matching the input.

        @param      subj  (string) The relation subject
        @param      pred  (string) The relation predicate
        @param      obj   (string) The relation object

        @return     A list of relations
        """
        if pred != "" and subj != "" and obj != "" and subj != "-1" and obj != "-1":
            subj = self.get_element(subj)
            obj = self.get_element(obj)
            rels = self.get_reasoner_relations(subj, pred, obj)
            if rels is not None:
                return rels
            else:
                return subj.getRelations(pred=self.get_sub_properties(pred), obj=obj.id)
        msg = srvs.WmQueryRelations.Request()
        msg.relation = utils.relation2msg(utils.makeRelation(subj, pred, obj))
        res = self._call(self._query_relations, msg)
        if(res):
            return [utils.msg2relation(x) for x in res.matches]

    def check_relation(self, subj, pred, obj, state, abstract):
        """
        @brief      Check if a relation match the desired state

        @param      subj      (string) The relation subject
        @param      pred      (string) The relation predicate
        @param      obj       (string) The relation object
        @param      state     (bool) The state to match
        @param      abstract  (bool) If True and a template of the
                              subject/object instance is available, uses
                              that as subject/object

        @return     (bool)
        """
        if abstract:
            if subj.hasProperty("skiros:Template"):
                e1 = subj.getProperty("skiros:Template").value
            else:
                e1 = subj.id
            if obj.hasProperty("skiros:Template"):
                e2 = obj.getProperty("skiros:Template").value
            else:
                e2 = obj.id
            return (e2 in self.get_triples(e1, pred)) == state
        else:
            return bool(self.get_relations(subj.id, pred, obj.id)) == state

    def _resolve_elements2(self, keys, ph, verbose=False):
        """
        @brief      Find all possible inputs for one or more keys of a
                    skill parameter handler

        @param      keys     list(string) a key list pointing out the params to be
                             resolved
        @param      ph       (ParamHandler)
        @param      verbose  (bool) If true, prints out results

        @return     list(Element)
        """
        first = {}
        couples = {}
        for key in keys:
            first[key] = np.array(self.resolve_elements(ph.getParamValue(key)))
            if not first[key].any():
                log.warn("resolve_elements", "No input found for param {}. Resolving: {}".format(
                    key, ph.getParamValue(key).printState(True)))
        all_keys = list(ph.keys())
        coupled_keys = []
        overlap_keys = []
        relations_done = set([])
        # Build tuples of concording parameters
        for i in range(len(all_keys)):  # Loop over all keys
            key_base = all_keys[i]
            if not isinstance(ph.getParamValue(key_base), Element):
                continue
            for j in ph.getParamValue(key_base)._relations:  # Loop over relation constraints
                # print j
                if j["src"] == "-1":  # -1 is the special autoreferencial value
                    key2 = j["dst"]
                    key = key_base
                    rel_id = key_base + j["type"] + j["dst"]
                    if rel_id in relations_done:  # Skip relation with previous indexes, already considered
                        continue
                    else:
                        # print rel_id
                        relations_done.add(rel_id)
                else:
                    key2 = key_base
                    key = j["src"]
                    rel_id = j["src"] + j["type"] + key_base
                    if rel_id in relations_done:  # Skip relation with previous indexes, already considered
                        continue
                    else:
                        # print rel_id
                        relations_done.add(rel_id)
                # Check necessary because at the moment ._relations contains a mix Toclean
                if not ph.hasParam(key) or not ph.hasParam(key2):
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
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if self.check_relation(
                        e1, j["type"], e2, j['state'], j['abstract'])]
                    if temp:
                        try:
                            couples[(key, key2)] = np.concatenate(couples[(key, key2)], np.array(temp))
                        except BaseException:
                            log.error("", "MERGING: {} and {} ".format(couples[(key, key2)], np.array(temp)))
                    else:
                        log.warn("resolve_elements", "No input for params {} {}. No match for: {} {} {}".format(
                            key, key2, set1, j["type"], set2))
                else:
                    if key in coupled_keys:
                        overlap_keys.append(key)
                    else:
                        coupled_keys.append(key)
                    if key2 in coupled_keys:
                        overlap_keys.append(key2)
                    else:
                        coupled_keys.append(key2)
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if self.check_relation(
                        e1, j["type"], e2, j['state'], j['abstract'])]
                    couples[(key, key2)] = np.array(temp)
                    if not temp:
                        log.warn("resolve_elements", "No input for params {} {}. No match for: {} {} {}".format(
                            key, key2, set1, j["type"], set2))
        # Merge the tuples with an overlapping key
        if overlap_keys:
            loop = True
            iters = 15
            while loop:  # Iterate until no shared keys are found
                iters -= 1
                if iters == 0:
                    raise BaseException("Reached maximum number of iterations when merging keys")
                loop = False
                coupled_keys2 = []
                merged = {}
                # print 'qui:'
                for k1, s1 in couples.items():
                    for k2, s2 in couples.items():
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
                        for k1, s1 in couples.items():
                            if key in k1:
                                merged[k1] = s1
                couples = merged
        # Add back keys that are not coupled to others
        for key in keys:
            if not key in coupled_keys:
                couples[key] = first[key]
        if verbose:
            for k, v in couples.items():
                s = "{}:".format(k)
                for i in v:
                    if not isinstance(i, Element):
                        s += "["
                        for j in i:
                            s += "{},".format(j)
                        s += "]"
                    else:
                        s += "{},".format(i)
                print(s)
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
        # print keys
        sets = []
        # print c
        # print d
        for v1 in s1:
            for v2 in s2:
                append = True
                for i in range(len(shared_k)):
                    # print str(v1[a[i]].printState()) + 'vs' + str(v1[b[i]].printState()) + '=' + str(v1[a[i]]!=v2[b[i]])
                    if v1[a[i]] != v2[b[i]]:
                        append = False
                if append:
                    sets.append(np.array(self._concatenate(v1[c], v2[d])))
        return tuple(keys), np.array(sets)
