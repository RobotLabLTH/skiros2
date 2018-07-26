from os import path
import skiros2_common.tools.logger as log
import skiros2_common.ros.utils as utils
from skiros2_common.core.world_element import Element
from skiros2_world_model.ros.ontology_server import Ontology
import rdflib
from rdflib.namespace import RDF, RDFS, OWL, XSD
from wrapt.decorators import synchronized
from skiros2_common.tools.id_generator import IdGen

scene_context = "scene"

class WorldModel(Ontology):
    def __init__(self, verbose, change_cb):
        Ontology.__init__(self)
        self.wm = self.add_context(scene_context)
        self._verbose = verbose
        self._id_gen = IdGen()
        self._reasoners = {}
        self._change_cb = change_cb

    def set_workspace(self, workspace):
        self._workspace = workspace

    def reset(self, add_root=True, scene_name="skiros:blank_scene"):
        """
        @brief Initialize the scene
        """
        self.ontology().remove_context(self.wm)
        self.wm = self.add_context(scene_context)
        self._id_gen.clear()
        self._elements_cache = dict()
        if add_root:
            if self.has_individual(scene_name):
                root = self.get_individual(scene_name)
            else:
                root = Element("skiros:Scene", scene_name, 0)
            self.add_element(root, self.__class__.__name__)

    def _set(self, statement, author, time=None, probability=1.0):
        """
        @brief Remove any existing triples for subject and predicate before adding
        (subject, predicate, object).

        Convenience method to update the value of object
        """
        if self._verbose:
            log.info(author, log.logColor.RED + log.logColor.BOLD  + "[-] ({}) - ({}) - (*))".format(self.uri2lightstring(statement[0]), self.uri2lightstring(statement[1])))
            log.info(author, log.logColor.GREEN + log.logColor.BOLD  + "[+] ({}) - ({}) - ({})".format(self.uri2lightstring(statement[0]), self.uri2lightstring(statement[1]), self.uri2lightstring(statement[2])))
        self.wm.set(statement)
        if self._elements_cache.has_key(self.uri2lightstring(statement[0])):
            del self._elements_cache[self.uri2lightstring(statement[0])]

    def _remove(self, statement, author, is_relation=False, time=None, probability=1.0):
        """
        @brief Remove a statement from the scene
        """
        if self._verbose:
            log.info(author, log.logColor.RED + log.logColor.BOLD  + "[-] ({}) - ({}) - ({})".format(self.uri2lightstring(statement[0]), self.uri2lightstring(statement[1]), self.uri2lightstring(statement[2])))
        self.wm.remove(statement)
        if is_relation:
            if self._elements_cache.has_key(self.uri2lightstring(statement[0])):
                del self._elements_cache[self.uri2lightstring(statement[0])]
            if self._elements_cache.has_key(self.uri2lightstring(statement[2])):
                del self._elements_cache[self.uri2lightstring(statement[2])]
            self._change_cb(author, "remove", relation={'src': self.uri2lightstring(statement[0]), 'type': self.uri2lightstring(statement[1]), 'dst': self.uri2lightstring(statement[2])})

    def _add(self, statement, author, is_relation=False, time=None, probability=1.0):
        """
        @brief Add a statement to the scene
        """
        if self._verbose:
            log.info(author, log.logColor.GREEN + log.logColor.BOLD  + "[+] ({}) - ({}) - ({})".format(self.uri2lightstring(statement[0]), self.uri2lightstring(statement[1]), self.uri2lightstring(statement[2])))
        self.wm.add(statement)
        if is_relation:
            if self._elements_cache.has_key(self.uri2lightstring(statement[0])):
                del self._elements_cache[self.uri2lightstring(statement[0])]
            if self._elements_cache.has_key(self.uri2lightstring(statement[2])):
                del self._elements_cache[self.uri2lightstring(statement[2])]
            self._change_cb(author, "add", relation={'src': self.uri2lightstring(statement[0]), 'type': self.uri2lightstring(statement[1]), 'dst': self.uri2lightstring(statement[2])})

    def _element2statements(self, e):
        to_ret = []
        subject = self.lightstring2uri(e.id)
        to_ret.append(((subject, RDF.type, OWL.NamedIndividual), False))
        to_ret.append(((subject, RDF.type, self.lightstring2uri(e.type)), False))
        to_ret.append(((subject, RDFS.label, rdflib.term.Literal(e.label)), False))
        for k, p in e._properties.iteritems():
            predicate = self.lightstring2uri(k)
            for v in p.getValues():
                value = rdflib.term.Literal(v, datatype=self._getDatatype(p))
                to_ret.append(((subject, predicate, value), False))
        for r in list(e._relations):
            if r['src']=="-1" or r['src']==e.id:
                if not self.exists_in_ontology(self.lightstring2uri(r['dst'])):
                    log.error("[element2statements]", "Element with key {} is not defined in ontology. Skipped relation: {}".format(r['dst'], r))
                    e.remove_relation(r)
                    continue
                to_ret.append(((subject, self.lightstring2uri(r['type']), self.lightstring2uri(r['dst'])), True))
            else:
                if not self.exists_in_ontology(self.lightstring2uri(r['src'])):
                    log.error("[element2statements]", "Element with key {} is not defined in ontology. Skipped relation: {}".format(r['src'], r))
                    e.remove_relation(r)
                    continue
                to_ret.append(((self.lightstring2uri(r['src']), self.lightstring2uri(r['type']), subject), True))
        return to_ret

    def _uri2type(self, uri):
        return uri.split('-')[0]

    def _uri2id(self, uri):
        if uri.find('-')<0:
            return -1
        return int(uri.split('-')[1])

    def _get_types(self, eclass):
        """
        Return all scene elements of a type
        """
        to_ret = []
        types_filter = self.get_sub_classes(eclass)
        for etype in types_filter:
            for subj in self.wm.subjects(RDF.type, self.lightstring2uri(etype)):
                to_ret.append(self.get_element(self.uri2lightstring(subj)))
        return to_ret

    def _add_reasoners_prop(self, e):
        """
        if the object is marked as associated to a reasoner, check that all properties are present
        """
        if e.hasProperty("skiros:DiscreteReasoner"):
            for v in e.getProperty("skiros:DiscreteReasoner").values:
                if self._reasoners.has_key(v):
                    self._reasoners[v].addProperties(e)

    def _stop_reasoners(self):
        for _, r in self._reasoners.iteritems():
            r.stop()

    def _start_reasoners(self):
        for _, r in self._reasoners.iteritems():
            r.execute()

    def get_reasoner(self, reasoner_class):
        if not self._reasoners.has_key(reasoner_class):
            return None
        return self._reasoners[reasoner_class]

    def load_reasoner(self, reasoner_class):
        self._reasoners[reasoner_class.__name__] = reasoner_class()
        self._reasoners[reasoner_class.__name__].init(self)
        self._reasoners[reasoner_class.__name__].execute()
        log.info("[load_reasoner] Loaded {}".format(reasoner_class.__name__))

    def has_individual(self, name):
        """
        @brief Returns true if the individual exists in the ontology
        """
        subject = self.lightstring2uri(name)
        if self.ontology().value(subject, RDF.type):
            return True
        return False

    @synchronized
    def get_individual(self, name, context_id=None):
        """
        @brief Builds an element from an individual
        """
        subject = self.lightstring2uri(name)
        if not self.ontology(context_id).value(subject, RDF.type):
            raise Exception("Element {} doesn't exist in ontology. Uri: {}. Context: {}".format(name, subject, context_id))
        e = Element()
        for predicate, obj in self.ontology(context_id).predicate_objects(subject):
            if OWL.DatatypeProperty in self.ontology().objects(predicate, RDF.type) or predicate==RDFS.comment:
                e.setProperty(self.uri2lightstring(predicate), obj.value, self.uri2lightstring(obj.datatype), force_convertion=True)
            elif OWL.ObjectProperty in self.ontology().objects(predicate, RDF.type):
                e.addRelation("-1", self.uri2lightstring(predicate), self.uri2lightstring(obj))
            elif predicate==RDF.type and obj!=OWL.NamedIndividual:
                e._type = self.uri2lightstring(str(obj))
            elif obj==OWL.NamedIndividual:
                pass
            elif predicate==RDFS.label:
                e._label = obj.value
            else:
                log.error("[get_individual]", "Ignoring {}-{}-{}. Predicate is not defined in the ontology.".format(name, self.uri2lightstring(predicate), self.uri2lightstring(obj)))
        for subj, predicate in self.ontology(context_id).subject_predicates(subject):
            e.addRelation(self.uri2lightstring(subj), self.uri2lightstring(predicate), "-1")
        self._add_reasoners_prop(e)
        return e

    def get_template_individual(self, name):
        """
        @brief Builds an element from an ontology individual
        """
        e = self.get_individual(name)
        if e:
            e.label = name
            e.setProperty("skiros:Template", name)
        return e

    def is_scene_element(self, uri):
        return self._uri2id(uri) >= 0

    def exists_in_ontology(self, uri, context_id=None):
        return bool(self.ontology(context_id).value(uri, RDF.type))

    @synchronized
    def load_scene(self, filename):
        """
        @brief Load scene from file
        """
        if not path.isfile(self._workspace+"/"+filename):
            log.error("[load_scene]", "Can't load scene {}. File not found. ".format(filename))
            return
        self._stop_reasoners()
        self.reset(add_root=False)
        self.wm.parse(self._workspace+"/"+filename, format='turtle')
        individuals = self.wm.query("SELECT ?x WHERE { ?x rdf:type <http://www.w3.org/2002/07/owl#NamedIndividual>. } ")
        for i in individuals:
            i = self.uri2lightstring(i[0])
            iid = self._uri2id(i)
            if iid>=0:
                self._id_gen.getId(iid)
        self._start_reasoners()
        log.info("[load_scene]", "Loaded scene {}. ".format(filename))

    @synchronized
    def save_scene(self, filename):
        """
        @brief Save scene to file
        """
        self.wm.serialize(self._workspace+"/"+filename, format='turtle')

    @synchronized
    def add_relation(self, r, author, is_relation):
        """
        @brief Add an rdf triple
        """
        s = self.lightstring2uri(r['src'])
        o = self.lightstring2uri(r['dst'])
        self._add((s, self.lightstring2uri(r['type']), o), author, is_relation=is_relation)

    @synchronized
    def remove_relation(self, r, author, is_relation):
        """
        @brief Remove an rdf triple
        """
        s = self.lightstring2uri(r['src'])
        o = self.lightstring2uri(r['dst'])
        self._remove((s, self.lightstring2uri(r['type']), o), author, is_relation=is_relation)

    def get_relations(self, r):
        to_ret = []
        triples = self.wm.triples((self.lightstring2uri(r['src']), self.lightstring2uri(r['type']), self.lightstring2uri(r['dst'])))
        for s, p, o in triples:
            to_ret.append(utils.makeRelation(self.uri2lightstring(s), self.uri2lightstring(p), self.uri2lightstring(o)))
        return to_ret

    def get_element(self, uri):
        """
        @brief Get an element from the scene
        """
        if self._elements_cache.has_key(uri):
            return self._elements_cache[uri]
        e = self.get_individual(uri, self.wm)
        e._id = uri
        self._elements_cache[e.id] = e
        return e

    def _getDatatype(self, param):
        if param.dataTypeIs(str) or param.dataTypeIs(unicode):
            return XSD.string
        elif param.dataTypeIs(float):
            return XSD.float
        elif param.dataTypeIs(bool):
            return XSD.boolean
        elif param.dataTypeIs(int):
            return XSD.integer
        else:
            log.error("[Wm]", "Param {} has type {} that is not supported.".format(param.key, param.dataType))
            return None

    @synchronized
    def add_element(self, e, author):
        """
        @brief Add an element to the scene
        """
        e.setUri(self._id_gen.getId(e.getIdNumber()))
        for name, r in self._reasoners.iteritems():
            if not r.parse(e, "add"):
                raise Exception("Reasoner {} rejected the element {} add".format(name, e))
        if e.hasProperty("skiros:Template"):
            e.addRelation("-1", "skiros:hasTemplate", e.getProperty("skiros:Template").value)
        statements = self._element2statements(e)
        for s, is_relation in statements:
            self._add(s, author, is_relation)
        self._elements_cache[e.id] = e
        return e

    @synchronized
    def update_element(self, e, author):
        """
        @brief Update an element in the scene
        """
        if not self._id_gen.hasId(self._uri2id(e.id)):
            log.error("[update_element]", "Request update from {}, but Id {} is not present in the wm. ".format(author, self._uri2id(e.id)))
            return
        for name, r in self._reasoners.iteritems():
            if not r.parse(e, "update"):
                raise Exception("Reasoner {} rejected the element {} update".format(name, e))
        prev = self._element2statements(self.get_element(e.id))
        curr = self._element2statements(e)
        c1, c2 = zip(*curr)
        for s, is_relation in prev:
            if not s in c1:
                self._remove(s, author, is_relation)
        p1, p2 = zip(*prev)
        for s, is_relation in curr:
            if not s in p1:
                self._add(s, author, is_relation)
        self._elements_cache[e.id] = e

    @synchronized
    def update_properties(self, e, author, reasoner=None, publish=True):
        """
        @brief Update properties of an element in the scene
        """
        if not self._id_gen.hasId(self._uri2id(e.id)):
            log.error("[update_element]", "Request update from {}, but Id {} is not present in the wm. ".format(author, self._uri2id(e.id)))
            return
        for name, r in self._reasoners.iteritems():
            if not r.parse(e, "update"):
                raise Exception("Reasoner {} rejected the element {} update".format(name, e))
        old_e = self.get_element(e.id)
        subject = self.lightstring2uri(e.id)
        if reasoner is not None:
            prop_to_update = reasoner.getAssociatedData()
        else:
            prop_to_update = e.available_properties()
        for k in prop_to_update:
            predicate = self.lightstring2uri(k)
            p = e.getProperty(k)
            values = p.values
            if not old_e.hasProperty(k):
                old_e.setProperty(k, values)
                for i in range(0, len(values)):
                    self._add((subject, predicate, rdflib.term.Literal(values[i], datatype=self._getDatatype(p))), author)
            elif old_e.getProperty(k).values!=values:
                old_e.setProperty(k, values)
                if values:
                    self._set((subject, predicate, rdflib.term.Literal(values[0], datatype=self._getDatatype(p))), author)
                for i in range(1, len(values)):
                    self._add((subject, predicate, rdflib.term.Literal(values[i], datatype=self._getDatatype(p))), author)
        self._elements_cache[e.id] = old_e
        if publish:
            self._change_cb(author, "update", old_e)

    @synchronized
    def resolve_elements(self, description):
        """
        @brief Return all elements matching the profile in input (type, label, properties)
        """
        first = []
        to_ret = []
        for e in self._get_types(description._type):
            if description._label=="" or description._label=="Unknown" or e._label==description._label:
                first.append(e)
        #Filter by properties
        for e in first:
            add = True
            for k, p in description._properties.iteritems():
                if not e.hasProperty(k):
                    add = False
                    break
                for v in p.values:
                    if v == "" or v==None:
                        break
                    if not v in e.getProperty(k).values:
                        add = False
                        break
                if not add:
                    break
            if add:
                to_ret.append(e)
        return to_ret

    @synchronized
    def remove_element(self, e, author):
        """
        Remove an element from the scene
        """
        e = self.get_element(e.id)
        self._id_gen.removeId(self._uri2id(e.id))
        for name, r in self._reasoners.iteritems():
            if not r.parse(e, "remove"):
                raise Exception("Reasoner {} rejected the element {} removal".format(name, e))
        statements = self._element2statements(e)
        for s, is_relation in statements:
            self._remove(s, author, is_relation)
        return e.id

    def _get_recursive(self, e, rels_filter, types_filter, elist):
        """
        Get all elements related to the initial one. Anti-loop guarded
        """
        elist[e.id] = e
        for r in e.getRelations("-1", rels_filter):
            if self.is_scene_element(r['dst']):
                e2 = self.get_element(r['dst'])
                if (e2._type in types_filter or not types_filter) and not elist.has_key(e2._id):
                    self._get_recursive(e2, rels_filter, types_filter, elist)

    @synchronized
    def get_recursive(self, eid, rel_filter="", type_filter=""):
        """
        Get an element from the scene and all elements related to the initial one
        """
        to_ret = {}
        rels_filter = []
        types_filter = []
        if rel_filter!="":
            rels_filter = self.get_sub_relations(rel_filter)
        if type_filter!="":
            types_filter = self.get_sub_classes(type_filter)
        self._get_recursive(self.get_element(eid), rels_filter, types_filter, to_ret)
        return to_ret

    def remove_recursive(self, e, author, rel_filter="", type_filter=""):
        """
        Remove an element from the scene and all elements related to the initial one
        """
        to_ret = []
        rels_filter = []
        types_filter = []
        to_ret.append(self.remove_element(e, author))
        if rel_filter!="":
            rels_filter = self.get_sub_relations(rel_filter)
        if type_filter!="":
            types_filter = self.get_sub_classes(type_filter)
        for r in e._relations:
            if r['src']=="-1":
                if (r['type'] in rels_filter or rel_filter=="") and self.is_scene_element(r['dst']):
                    e2 = self.get_element(r['dst'])
                    if e2._type in types_filter or type_filter=="":
                        to_ret += self.remove_recursive(e2, author, rel_filter, type_filter)
        return to_ret
