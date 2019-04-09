import skiros2_common.tools.logger as log
from skiros2_common.core.world_element import *
from skiros2_common.tools.id_generator import IdGen
import skiros2_common.core.params as params
import semanticnet as sn
from itertools import chain
from copy import copy, deepcopy
import numpy as np


class WorldModel:
    """
    This world model implementation is made to remain local and interface when necessary with the global wm
    """
    _id = 0
    _graph = sn.Graph()
    _types = {}

    def __init__(self, wmi, scene_name=None):
        self._id_gen = IdGen()
        self._wmi = wmi
        self._keep_sync = False
        self._verbose = False
        self.reset(scene_name)

    def __copy__(self):
        wm = WorldModel(self._wmi)
        wm._verbose = self._verbose
        wm._id = self._id
        wm._graph = self._graph
        wm._types = self._types
        return wm

    def __deepcopy__(self, memo):
        result = self.__copy__()
        memo[id(self)] = result
        return result

    def __enter__(self):
        self.syncKeep(True)
        return self

    def __exit__(self, type, value, traceback):
        self.syncKeep(False)

    def reset(self, scene_name=None):
        self._id = 0
        self._graph = sn.Graph()
        self._types = dict()
        if scene_name:
            root = Element(":Scene", scene_name, 0)
            props = {"type": root._type, "label": root._label}
            self._graph.add_node(dict(chain(props.items(), root._properties.items())), root._id)

    def _addType(self, etype, eid):
        if etype not in self._types:
            self._types[etype] = []
        self._types[etype].append(eid)

    def _removeType(self, etype, eid):
        try:
            self._types[etype].remove(eid)
        except BaseException:
            log.error("_removeType", "No element id: {} type: {}".format(eid, etype))

    def _getTypes(self, etype):
        """
        Fast retrieval of elements of same type
        """
        to_ret = []
        if etype in self._types:
            to_ret += [self._graph.get_node(t) for t in self._types[etype] if self._graph.has_node(t)]
        for c in self._wmi.get_sub_classes(etype, True):
            if c in self._types:
                to_ret += [self._graph.get_node(t) for t in self._types[c] if self._graph.has_node(t)]
        #print "{} {}".format(etype, len(to_ret))
        return to_ret

    def syncKeep(self, value=True):
        """
        When set, the modifications are syncronized with main wm
        """
        self._keep_sync = value

    def pushElement(self, e, action):
        """
        Update element to main wm
        """
        if self._keep_sync:
            #print "Pushing {} {}".format(e.printState(), action)
            if action == "add":
                self._wmi.add_element(e)
            elif action == "update":
                e._relations = self.getContextRelations(e)
                self._wmi.update_element(e)
            elif action == "remove":
                self._wmi.remove_element(e)

    def pushRelation(self, sub, rel, obj, value):
        """
        Update relation to main wm
        """
        if self._keep_sync:
            #print "Pushing {} {} {} {}".format(sub, rel, obj, value)
            self._wmi.set_relation(sub, rel, obj, value)

    def sync(self):
        """
        Pull the graph from the main world model
        """
        self.importGraph(self._wmi.get_branch("skiros:Scene-0"))

    def importGraph(self, elements):
        self.reset()
        for e in elements:
            self._addNode(e)
        for e in elements:
            for r in e._relations:
                try:
                    if r['src'] == "-1":
                        self._addEdge(e._id, r['type'], r['dst'])  # NOTE: i have to skip passive relations...this could create problems
                except BaseException:
                    log.error("[importGraph]", "Skipping relation {}. The child node was not imported.".format(r))
                    continue

    def importRelations(self, relations):
        for r in relations:
            self.set_relation(*r)

    def _printRecursive(self, root, indend, relation_filter):
        s = root.printState()
        print indend + s
        indend = "-" * (len(indend) + len(s)) + "->"
        for e in self.getChildren(root._id, relation_filter):  # sceneProperty
            self._printRecursive(e, indend, relation_filter)

    def printModel(self, relation_filter="skiros:sceneProperty"):
        root = self.get_element("skiros:Scene-0")
        #print str(self._graph)
        self._printRecursive(root, "", relation_filter)
        # nx.draw(self._graph.networkx_graph())
        return

    def getAbstractElement(self, etype, elabel):
        e = Element(etype, elabel)
        self.add_element(e, 0, 'hasAbstract')
        return e

    def resolve_elements2(self, keys, ph):
        """
        Return all elements matching the profile in input (type, label, properties and relations)

        Keys: a key list pointing out the params to be resolved
        ph: a ParamHandler class

        """
        first = {}
        couples = {}
        print_out = False
        for key in keys:
            first[key] = np.array(self.resolve_element(ph.getParamValue(key)))
            if not first[key].any():
                log.warn("resolve_elements", "No input found for param {}. Resolving: {}".format(key, ph.getParamValue(key).printState(True)))
        all_keys = [key for key, _ in ph._params.iteritems()]
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
                #print "{} {}".format(key, key2)
                if this.getIdNumber() >= 0 and other.getIdNumber() >= 0:  # If both parameters are already set, no need to resolve..
                    continue
                if this.getIdNumber() >= 0:
                    set1 = [this]
                else:
                    if ph.getParam(key).paramType() == params.ParamTypes.Optional:
                        continue
                    else:
                        set1 = first[key]
                if other.getIdNumber() >= 0:
                    set2 = [other]
                else:
                    if ph.getParam(key2).paramType() == params.ParamTypes.Optional:
                        continue
                    else:
                        set2 = first[key2]
                if (key, key2) in couples:
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if bool(self.get_relations(e1._id, j["type"], e2._id)) == j['state']]
                    if temp:
                        couples[(key, key2)] = np.concatenate(couples[(key, key2)], np.array(temp))
                    else:
                        log.warn("resolve_elements", "No input for params {} {}. Resolving: {} {}".format(key, key2, ph.getParamValue(key).printState(True), ph.getParamValue(key2).printState(True)))
                else:
                    if key in coupled_keys:
                        overlap_keys.append(key)
                    else:
                        coupled_keys.append(key)
                    if key2 in coupled_keys:
                        overlap_keys.append(key2)
                    else:
                        coupled_keys.append(key2)
                    temp = [np.array([e1, e2]) for e1 in set1 for e2 in set2 if bool(self.get_relations(e1._id, j["type"], e2._id)) == j['state']]
                    couples[(key, key2)] = np.array(temp)
                    if not temp:
                        log.warn("resolve_elements", "No input for params {} {}. Resolving: {} {}".format(key, key2, ph.getParamValue(key).printState(True), ph.getParamValue(key2).printState(True)))
        # Merge the tuples with an overlapping key
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

    def resolve_element(self, description):
        """
        Return all elements matching the profile in input (type, label, properties)
        """
        first = []
        to_ret = []
        #print 'description ' + description.printState(True)
        # Get all nodes matching type and label
        #print get_sub_classes(STMN[description._type], True)
        for e in self._getTypes(description._type):
            if description._label == "" or description._label == "Unknown" or e['label'] == description._label:
                first.append(self._makeElement(e))
        # Filter by properties
        for e in first:
            add = True
            for k, p in description._properties.iteritems():
                if not e.hasProperty(k):
                    add = False
                    break
                for v in p.getValues():
                    if v == "" or v is None:
                        break
                    if e.getProperty(k).find(v) < 0:
                        add = False
                        break
                if not add:
                    break
            if add:
                to_ret.append(e)
        #print to_ret
        return to_ret

    def _makeElement(self, props):
        e = Element()
        copy = deepcopy(props)
        e._id = copy.pop("id")
        e._type = copy.pop("type")
        e._label = copy.pop("label")
        e._properties = copy
        return e

    def get_element(self, eid):
        try:
            eprops = self._graph.get_node(eid)
        except KeyError:
            raise KeyError("{} not found. Debug: {} {}".format(eid, self._graph, self._types))
        return self._makeElement(eprops)

    def _addNode(self, element):
        element.setUri(self._id_gen.getId(element.getIdNumber()))
        if self._verbose:
            log.debug('add', str(element._id))
        props = {"type": element._type, "label": element._label}
        self._graph.add_node(dict(chain(props.items(), element._properties.items())), element._id)
        self._addType(element._type, element._id)

    def _resolve_local_relations(self, e, lr):
        for r in lr:
            sub_e = r['dst']
            sub_e.addRelation(e._id, r['type'], "-1")
            if sub_e._id == "":
                if self.add_element2(sub_e) < 0:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to add local element {}".format(sub_e))
            else:
                if self.update_element(sub_e) < 0:
                    log.error("[{}]".format(self.__class__.__name__), "Failed to update local element {}".format(sub_e))

    def add_element2(self, element):
        lr = copy(element._local_relations)
        element._local_relations = list()
        self.pushElement(element, "add")
        self._addNode(element)
        for r in element._relations:
            if r['src'] == "-1":
                self.set_relation(element._id, r['type'], r['dst'], True, push=False)
            else:
                self.set_relation(r['src'], r['type'], element._id, True, push=False)
        self._resolve_local_relations(element, lr)
        return element._id

    def add_element(self, element, parent_id, relation):
        self.pushElement(element, "add")
        self._addNode(element)
        self.set_relation(parent_id, relation, element._id, True, push=False)
        return element._id

    def update_element(self, element):
        for r in element._relations:
            if r['src'] == "-1":
                self.set_relation(element._id, r['type'], r['dst'], True, push=False)
            else:
                self.set_relation(r['src'], r['type'], element._id, True, push=False)
        self.pushElement(element, "update")
        if not self._graph.has_node(element._id):
            log.warn("update_element", "No element found with key {}".format(element._id))
            return
        props = {"type": element._type, "label": element._label}
        self._graph.add_node(dict(chain(props.items(), element._properties.items())), element._id)
        return element._id

    def remove_element(self, eid):
        if self._verbose:
            log.debug('remove', str(eid))
        self.pushElement(eid, "remove")
        eprops = self._graph.get_node(eid)
        self._removeType(eprops["type"], eid)
        self._graph.remove_node(eid)

    def _check_relation(self, esubject, relation, eobject, value, push):
        """
        Remove the old contain relation, to maintain the tree structure
        """
        if(self.isRelationType(relation, "skiros:sceneProperty") and value):
            self.set_relation("-1", "skiros:sceneProperty", eobject, False, push)

    def isElementType(self, etype, abstract_type):
        return etype == abstract_type or (self._wmi.addPrefix(etype) in self._wmi.get_sub_classes(abstract_type, True))

    def isRelationType(self, relation, rtype="skiros:sceneProperty"):
        #print "{}={} is {}".format(relation, rtype, relation==rtype or self._wmi.addPrefix(relation) in self._wmi.get_sub_properties(rtype, True))
        return relation == rtype or self._wmi.addPrefix(relation) in self._wmi.get_sub_properties(rtype, True)

    def _addEdge(self, esubject, relation, eobject):
        if self._verbose:
            log.debug('add', str(esubject) + "-" + relation + "-" + str(eobject))
        self._graph.add_edge(esubject, eobject, {"type": relation})

    def set_relation(self, esubject, relation, eobject, value=True, push=True):
        if self.get_relations(esubject, relation, eobject) and value:  # Avoid adding twice the same statement
            return True
        self._check_relation(esubject, relation, eobject, value, push)
        try:
            if value:
                self._addEdge(esubject, relation, eobject)
            else:
                for e in self.get_relations(esubject, relation, eobject, True):
                    self._graph.remove_edge(e)
            if push:
                self.pushRelation(esubject, relation, eobject, value)
        except BaseException:
            self.printModel()
            raise
        return True

    def getAssociatedReasoner(self, relation):
        for cls in DiscreteReasoner.__subclasses__():
            instance = cls()
            if relation in instance.getAssociatedRelations():
                return instance
        return None

    def get_relations(self, esubject, relation, eobject, getId=False):
        rel = []
        for _, edge in self._graph.get_edges().items():
            if (esubject == "" or edge['src'] == esubject) and (eobject == "" or edge['dst'] == eobject) and (relation == "" or self.isRelationType(edge['type'], relation)):
                if getId:
                    rel.append(edge['id'])
                else:
                    new_edge = deepcopy(edge)
                    rel.append(new_edge)
        if not getId and relation != "" and esubject != "" and eobject != "":
            try:
                s = self.get_element(esubject)
                o = self.get_element(eobject)
                reasoner = s._getReasoner(relation)
                if relation in reasoner.computeRelations(s, o):
                    rel.append({"src": esubject, "type": relation, "dst": eobject})
            except KeyError:
                pass
        return rel

    def getContextRelations(self, esubject):
        """
        Get all relations related to a subject
        """
        rel = []
        for _, edge in self._graph.get_edges().items():
            if edge['src'] == esubject._id:
                new_edge = deepcopy(edge)
                del new_edge['id']
                new_edge['src'] = "-1"
                rel.append(new_edge)
            elif edge['dst'] == esubject._id:
                new_edge = deepcopy(edge)
                del new_edge['id']
                new_edge['dst'] = "-1"
                rel.append(new_edge)
        return rel

    def getChildren(self, eid, relation="skiros:sceneProperty"):
        to_ret = []
        for edge in self.get_relations(eid, relation, ""):
            e = self.get_element(edge['dst'])
            to_ret.append(e)
        return to_ret

    def getParent(self, eid):
        for edge in self.get_relations("", "skiros:sceneProperty", eid, getReasonersRel=False):
            return self.get_element(edge['src'])
