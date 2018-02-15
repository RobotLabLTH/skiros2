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

from os import walk, path
import rospy
import skiros2_common.tools.logger as log
import skiros2_common.ros.utils as utils
from skiros2_common.core.world_element import Element
from skiros2_world_model.ros.ontology_server import Ontology
import rospkg
import rdflib
from rdflib.namespace import RDF, RDFS, OWL, XSD
from wrapt.decorators import synchronized
from skiros2_common.tools.id_generator import IdGen

class WorldModel(Ontology):
    def __init__(self, verbose=False):
        self._verbose = verbose
        self._id_gen = IdGen()
        self._reasoners = {}
        self._ontology = rdflib.Graph()
        self._wm = rdflib.Graph()
        self._init()
        self._reset()
        init_scene = rospy.get_param('~init_scene', "")
        if init_scene!="":
            self.loadScene(init_scene)

    def _bind(self, prefix, uri):
        self._ontology.namespace_manager.bind(prefix, uri, True, True)
        self._wm.namespace_manager.bind(prefix, uri, True, True)

    def _init(self):
        rospack = rospkg.RosPack()
        self._skiros_dir = rospack.get_path('skiros2')+'/owl'
        self._workspace = rospy.get_param('~workspace_dir', self._skiros_dir)
        log.info("[{}]".format(self.__class__.__name__), "Workspace folder: {}".format(self._workspace))
        for (dirpath, dirnames, filenames) in walk(self._skiros_dir):
            for name in filenames:
                if name.find('.owl')>=0:
                    self.load(dirpath+'/'+name)
        for (dirpath, dirnames, filenames) in walk(self._workspace):
            for name in filenames:
                if name.find('.owl')>=0:
                    self.load(dirpath+'/'+name)
        #TODO: configure prefixes in launch file
        subjects = self._ontology.subjects(RDF.type, OWL.Ontology)
        for s in subjects:
            prefix = s[s.rfind("/")+1:].lower()
            if prefix.rfind(".")!=-1:
                prefix = prefix[:prefix.rfind(".")]
            s = s+"#"
            self._bind(prefix, s)
            log.info("[{}]".format(self.__class__.__name__), "Set prefix: {} for ontology: {}".format(prefix, s))
        self.setDefaultPrefix('skiros', 'http://rvmi.aau.dk/ontologies/skiros.owl#')
        for prefix, uri1 in self._ontology.namespace_manager.store.namespaces():
            if prefix.find("default")>-1:
                self._bind(prefix, "")
        self._bind("", "")
        #self.load(self._workspace+'learned_concepts.owl')

    def _reset(self, scene_name="skiros:aau_lab"):
        """
        Initialize the scene
        """
        self._id_gen.clear()
        self._elements_cache = dict()
        if self.hasIndividual(scene_name):
            root = self.getIndividual(scene_name)
        else:
            root = Element("skiros:Scene", scene_name, 0)
        self.addElement(root, self.__class__.__name__)

    def _remove(self, statement, author, time=None, probability=1.0):
        """
        Remove a statement from the scene and from ontology
        """
        if self._verbose:
            log.ok("[-] ({}) - ({}) - ({}) by: {}".format(self.uri2lightstring(statement[0]), self.uri2lightstring(statement[1]), self.uri2lightstring(statement[2]), author))
        self._ontology.remove(statement)
        self._wm.remove(statement)

    def _add(self, statement, author, time=None, probability=1.0):
        """
        Add a statement to the scene and the ontology
        """
        if self._verbose:
            log.ok("[+] ({}) - ({}) - ({}) by: {}".format(self.uri2lightstring(statement[0]), self.uri2lightstring(statement[1]), self.uri2lightstring(statement[2]), author))
        self._ontology.add(statement)
        self._wm.add(statement)

    def _element2statements(self, e):
        to_ret = []
        subject = self.lightstring2uri(e.id)
        to_ret.append((subject, RDF.type, OWL.NamedIndividual))
        to_ret.append((subject, RDF.type, self.lightstring2uri(e._type)))
        to_ret.append((subject, RDFS.label, rdflib.term.Literal(e._label)))
        for k, p in e._properties.iteritems():
            predicate = self.lightstring2uri(k)
            for v in p.getValues():
                value = rdflib.term.Literal(v)
                to_ret.append((subject, predicate, value))
        for r in e._relations:
            if r['src']=="-1" or r['src']==e.id:
                to_ret.append((subject, self.lightstring2uri(r['type']), self.lightstring2uri(r['dst'])))
            else:
                to_ret.append((self.lightstring2uri(r['src']), self.lightstring2uri(r['type']), subject))
        return to_ret

    def _uri2type(self, uri):
        return uri.split('-')[0]

    def _uri2id(self, uri):
        if uri.find('-')<0:
            return -1
        return int(uri.split('-')[1])

    def _getTypes(self, eclass):
        """
        Return all scene elements of a type
        """
        to_ret = []
        types_filter = self.getSubClasses(eclass)
        for etype in types_filter:
            for subj in self._ontology.subjects(RDF.type, self.lightstring2uri(etype)):
                subj = self.uri2lightstring(subj)
                if self.isSceneElement(subj):
                    to_ret.append(self.getElement(subj))
        return to_ret

    def _addReasonersProp(self, e):
        """
        if the object is marked as associated to a reasoner, check that all properties are present
        """
        if e.hasProperty("skiros:DiscreteReasoner"):
            for v in e.getProperty("skiros:DiscreteReasoner").values:
                if self._reasoners.has_key(v):
                    self._reasoners[v].addProperties(e)

    def _stopReasoners(self):
        for r in self._reasoners.items():
            r.stop()

    def _startReasoners(self):
        for r in self._reasoners.items():
            r.execute()

    def loadReasoner(self, reasoner_class):
        self._reasoners[reasoner_class.__name__] = reasoner_class()
        self._reasoners[reasoner_class.__name__].init(self)
        self._reasoners[reasoner_class.__name__].execute()
        log.info("[loadReasoner] Loaded {}".format(reasoner_class.__name__))

    def hasIndividual(self, name):
        """
        Returns true if the individual exists in the ontology
        """
        subject = self.lightstring2uri(name)
        if self._ontology.value(subject, RDF.type):
            return True
        return False

    @synchronized
    def getIndividual(self, name):
        """
        Builds an element from a long-term memory individual
        """
        subject = self.lightstring2uri(name)
        if not self._ontology.value(subject, RDF.type):
            raise Exception("Element {} doesn't exist in ontology. Uri: {}".format(name, subject))
        e = Element()
        for predicate, obj in self._ontology.predicate_objects(subject):
            if OWL.DatatypeProperty in self._ontology.objects(predicate, RDF.type) or predicate==RDFS.comment:
                e.setProperty(self.uri2lightstring(predicate), obj.value, self.uri2lightstring(obj.datatype), force_convertion=True)
            elif OWL.ObjectProperty in self._ontology.objects(predicate, RDF.type):
                e.addRelation("-1", self.uri2lightstring(predicate), self.uri2lightstring(obj))
            elif predicate==RDF.type and obj!=OWL.NamedIndividual:
                e._type = self.uri2lightstring(str(obj))
            elif obj==OWL.NamedIndividual:
                pass
            elif predicate==RDFS.label:
                e._label = obj.value
            else:
                log.error("[getIndividual]", "Ignoring {}-{}-{}. Predicate is not defined in the ontology.".format(name, self.uri2lightstring(predicate), self.uri2lightstring(obj)))
        for subj, predicate in self._ontology.subject_predicates(subject):
            e.addRelation(self.uri2lightstring(subj), self.uri2lightstring(predicate), "-1")
        self._addReasonersProp(e)
        return e

    def getTemplateIndividual(self, name):
        """
        Builds an element from a long-term memory individual
        """
        e = self.getIndividual(name)
        if e:
            e._label = name
            e.setProperty("skiros:Template", name)
        return e

    def isSceneElement(self, uri):
        return self._uri2id(uri) >= 0

    @synchronized
    def loadScene(self, filename):
        """
        Load scene from file
        """
        if not path.isfile(self._workspace+"/"+filename):
            log.error("[loadScene]", "Can't load scene {}. File not found. ".format(filename))
            return
        self._stopReasoners()
        self._ontology = self._ontology - self._wm
        self._wm = rdflib.Graph()
        self._wm.parse(self._workspace+"/"+filename, format='turtle')
        self._ontology = self._ontology + self._wm
        self._elements_cache = dict()
        self._id_gen.clear()
        individuals = self.query("SELECT ?x WHERE { ?x rdf:type <http://www.w3.org/2002/07/owl#NamedIndividual>. } ")
        for i in individuals:
            i = self.uri2lightstring(i[0])
            iid = self._uri2id(i)
            if iid>=0:
                self._id_gen.getId(iid)
        self._startReasoners()
        log.info("[loadScene]", "Loaded scene {}. ".format(filename))

    @synchronized
    def saveScene(self, filename):
        """
        Save scene to file
        """
        #ontology_id = rdflib.URIRef(filename)
        #self._wm.add((ontology_id, RDF.type, OWL.Ontology))
        #for s in self._ontology.subjects(object=OWL.Ontology):
        #    self._wm.add((ontology_id, OWL.imports, s))
        #print self._wm.serialize(format='turtle')
        self._wm.serialize(self._workspace+"/"+filename, format='turtle')

    @synchronized
    def addRelation(self, r, author):
        """
        @brief Add an rdf triple
        """
        s = self.lightstring2uri(r['src'])
        o = self.lightstring2uri(r['dst'])
        self._add((s, self.lightstring2uri(r['type']), o), author)
        if self._elements_cache.has_key(self.uri2lightstring(s)):
            del self._elements_cache[self.uri2lightstring(s)]
        if self._elements_cache.has_key(self.uri2lightstring(o)):
            del self._elements_cache[self.uri2lightstring(o)]

    @synchronized
    def removeRelation(self, r, author):
        """
        @brief Remove an rdf triple
        """
        s = self.lightstring2uri(r['src'])
        o = self.lightstring2uri(r['dst'])
        self._remove((s, self.lightstring2uri(r['type']), o), author)
        if self._elements_cache.has_key(self.uri2lightstring(s)):
            del self._elements_cache[self.uri2lightstring(s)]
        if self._elements_cache.has_key(self.uri2lightstring(o)):
            del self._elements_cache[self.uri2lightstring(o)]

    def getRelations(self, r):
        to_ret = []
        triples = self._wm.triples((self.lightstring2uri(r['src']), self.lightstring2uri(r['type']), self.lightstring2uri(r['dst'])))
        for s, p, o in triples:
            to_ret.append(utils.makeRelation(self.uri2lightstring(s), self.uri2lightstring(p), self.uri2lightstring(o)))
        return to_ret

    def getElement(self, uri):
        """
        @brief Get an element from the scene
        """
        if self._elements_cache.has_key(uri):
            return self._elements_cache[uri]
        e = self.getIndividual(uri)
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
    def addElement(self, e, author):
        """
        @brief Add an element to the scene
        """
        e.setUri(self._id_gen.getId(e.getIdNumber()))
        for name, r in self._reasoners.iteritems():
            if not r.parse(e, "add"):
                raise Exception("Reasoner {} rejected the element {} add".format(name, e))
        subject = self.lightstring2uri(e.id)
        self._add((subject, RDF.type, OWL.NamedIndividual), author)
        self._add((subject, RDF.type, self.lightstring2uri(e._type)), author)
        self._add((subject, RDFS.label, rdflib.term.Literal(e._label, datatype=XSD.string)), author)
        for k, p in e._properties.iteritems():
            predicate = self.lightstring2uri(k)
            for v in p.getValues():
                value = rdflib.term.Literal(v, datatype=self._getDatatype(p))
                self._add((subject, predicate, value), author)
        for r in e._relations:
            if r['src']=="-1":
                self._add((subject, self.lightstring2uri(r['type']), self.lightstring2uri(r['dst'])), author)
                if self._elements_cache.has_key(r['dst']):
                    del self._elements_cache[r['dst']]
            else:
                self._add((self.lightstring2uri(r['src']), self.lightstring2uri(r['type']), subject), author)
                if self._elements_cache.has_key(r['src']):
                    del self._elements_cache[r['src']]
        self._elements_cache[e.id] = e
        return e.id

    @synchronized
    def updateElement(self, e, author):
        """
        @brief Update an element in the scene
        """
        if not self._id_gen.hasId(self._uri2id(e.id)):
            log.error("[updateElement]", "Id {} is not present in the wm.".format(self._uri2id(e.id)))
            return
        for name, r in self._reasoners.iteritems():
            if not r.parse(e, "update"):
                raise Exception("Reasoner {} rejected the element {} update".format(name, e))
        prev = self.getContextStatements(e.id)
        curr = self._element2statements(e)
        for s in prev:
            if not s in curr:
                #print "Removing {}".format(s)
                self._remove(s, author)
        for s in curr:
            if not s in prev:
                #print "Adding {}".format(s)
                self._add(s, author)
        for r in e._relations: #Clear cache
            if r['src']=="-1":
                if self._elements_cache.has_key(r['dst']):
                    del self._elements_cache[r['dst']]
            else:
                if self._elements_cache.has_key(r['src']):
                    del self._elements_cache[r['src']]
        self._elements_cache[e.id] = e

    @synchronized
    def resolveElements(self, description):
        """
        @brief Return all elements matching the profile in input (type, label, properties)
        """
        first = []
        to_ret = []
        for e in self._getTypes(description._type):
            if description._label=="" or description._label=="Unknown" or e._label==description._label:
                first.append(e)
        #Filter by properties
        for e in first:
            add = True
            for k, p in description._properties.iteritems():
                if not e.hasProperty(k):
                    add = False
                    break
                for v in p.getValues():
                    if v == "" or v==None:
                        break
                    if not v in e.getPropertys(k):
                        add = False
                        break
                if not add:
                    break
            if add:
                to_ret.append(e)
        return to_ret

    @synchronized
    def removeElement(self, e, author):
        """
        Remove an element from the scene

        If recursive, remove also all elements related to the initial one
        """
        self._id_gen.removeId(self._uri2id(e.id))
        for name, r in self._reasoners.iteritems():
            if not r.parse(e, "remove"):
                raise Exception("Reasoner {} rejected the element {} removal".format(name, e))
        statements = self.getContextStatements(e.id)
        for s in statements:
            self._remove(s, author)
        if self._elements_cache.has_key(e.id):
            del self._elements_cache[e.id]
        return e.id

    def _getRecursive(self, e, rels_filter, types_filter, elist):
        """
        Get all elements related to the initial one. Anti-loop guarded
        """
        elist[e.id] = e
        for r in e.getRelations("-1", rels_filter):
            if self.isSceneElement(r['dst']):
                e2 = self.getElement(r['dst'])
                if (e2._type in types_filter or not types_filter) and not elist.has_key(e2._id):
                    self._getRecursive(e2, rels_filter, types_filter, elist)

    def getRecursive(self, eid, rel_filter="", type_filter=""):
        """
        Get an element from the scene and all elements related to the initial one
        """
        to_ret = {}
        rels_filter = []
        types_filter = []
        if rel_filter!="":
            rels_filter = self.getSubRelations(rel_filter)
        if type_filter!="":
            types_filter = self.getSubClasses(type_filter)
        self._getRecursive(self.getElement(eid), rels_filter, types_filter, to_ret)
        return to_ret

    def removeRecursive(self, e, author, rel_filter="", type_filter=""):
        """
        Remove an element from the scene and all elements related to the initial one
        """
        to_ret = []
        rels_filter = []
        types_filter = []
        to_ret.append(self.removeElement(e, author))
        if rel_filter!="":
            rels_filter = self.getSubRelations(rel_filter)
        if type_filter!="":
            types_filter = self.getSubClasses(type_filter)
        for r in e._relations:
            if r['src']=="-1":
                if (r['type'] in rels_filter or rel_filter=="") and self.isSceneElement(r['dst']):
                    e2 = self.getElement(r['dst'])
                    if e2._type in types_filter or type_filter=="":
                        to_ret += self.removeRecursive(e2, author, rel_filter, type_filter)
        return to_ret
