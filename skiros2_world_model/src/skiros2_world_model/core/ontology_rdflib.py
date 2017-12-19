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

import rdflib
from skiros2_common.core.world_element import Element
import skiros2_common.tools.logger as log
from rdflib.namespace import RDF, RDFS, OWL, XSD
import os.path

class Ontology:
    def __init__(self):
        self._ontology = rdflib.Graph()

    def setDefaultPrefix(self, prefix, uri):
        self._default_uri = self.addPrefix(prefix, uri)
        
    def addDefaultPrefix(self, uri):
        return rdflib.term.URIRef(self._default_uri[uri])
        
    def uri2lightstring(self, uri):
        if not uri:
            return uri
        if type(uri)==rdflib.URIRef:
            uri = uri.n3()
            uri = uri.replace('<', '')
            uri = uri.replace('>', '')
        if uri.find("#") < 0:
            return uri
        tokens = uri.split("#")
        for prefix, uri1 in self._ontology.namespaces():
            if tokens[0] == uri1[:-1]:
                return "{}:{}".format(prefix, tokens[1])
        return uri
            
    def lightstring2uri(self, name):
        if type(name)==rdflib.URIRef:
            return name
        if name=="":
            return None
        if name.find("#") > 0:
            return name
        if name.find(":") < 1:
            if name.find(":")==0:
                name = name[1:]
            return self.addDefaultPrefix(name)
        tokens = name.split(":")
        for prefix, uri in self._ontology.namespaces():
            if tokens[0] == prefix:
                return rdflib.term.URIRef("{}{}".format(uri, tokens[1]))
        return rdflib.term.URIRef(name)
                
    def createOntology(self, uri):
        new = rdflib.Graph()
        rdfterm = rdflib.URIRef(uri.split('.')[0])
        new.add((rdfterm, RDF.type, OWL.Ontology))
        for s in self._ontology.subjects(object=OWL.Ontology):
            new.add((rdfterm, OWL.imports, s))
        return new

    def load(self, ontology_uri):
        log.info("[{}]".format(self.__class__.__name__), "Loading ontology: {}".format(ontology_uri))
        return self._ontology.parse(ontology_uri)

    def save(self, file):
        self._ontology.serialize(destination=file, format='turtle')

    def query(self, query, cut_prefix=False):
        return self._ontology.query(query)

    def getContextStatements(self, name):
        """
        Returns all statements about an individual
        """
        to_ret = []
        uri = self.lightstring2uri(name)
        for s, p in self._ontology.subject_predicates(uri):
            to_ret.append((s, p, uri))
        for p, o in self._ontology.predicate_objects(uri):
            to_ret.append((uri, p, o))
        return to_ret
        
    def storeIndividual(self, e, file):
        if os.path.isfile(file):
            new = rdflib.Graph()
            new.parse(file, format='turtle')
        else:
            new = self.createOntology(file)
        subject = rdflib.URIRef(self.lightstring2uri(e._id))
        new.add((subject, RDF.type, OWL.NamedIndividual))
        new.add((subject, RDF.type, self.lightstring2uri(e._type)))
        for k, p in e._properties.iteritems():
            predicate = rdflib.URIRef(self.lightstring2uri(k))
            value = rdflib.Literal(p.getValues())
            new.add((subject, predicate, value))        
        new.serialize(destination=file, format='turtle')
        self._ontology += new
   
    def addRelation(self, r, author):
        self._ontology.add((self.lightstring2uri(r['src']), self.lightstring2uri(r['type']), self.lightstring2uri(r['dst'])))
        
    def removeRelation(self, r, author):
        self._ontology.remove((self.lightstring2uri(r['src']), self.lightstring2uri(r['type']), self.lightstring2uri(r['dst'])))
        
    def addPrefix(self, prefix, namespace):
        self._ontology.bind(prefix, namespace)
        return rdflib.Namespace(namespace)

    def removePrefix(self, parent_class):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def getType(self, uri):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def getSubClasses(self, parent_class, recursive=True):
        to_ret = []
        to_ret.append(parent_class)
        uri = self.lightstring2uri(parent_class)
        for subj in self._ontology.subjects(RDFS.subClassOf, uri):
            if recursive:
                to_ret += self.getSubClasses(self.uri2lightstring(subj), True)
            else:
                to_ret.append(self.uri2lightstring(subj))
        return to_ret

    def getSubProperties(self, parent_property="topDataProperty", recursive=True):
        to_ret = []
        to_ret.append(parent_property)
        uri = self.lightstring2uri(parent_property)
        for subj in self._ontology.subjects(RDFS.subPropertyOf, uri):
            if recursive:
                to_ret += self.getSubProperties(self.uri2lightstring(subj), True)
            else:
                to_ret.append(self.uri2lightstring(subj))
        return to_ret

    def getSubRelations(self, parent_property="topObjectProperty", recursive=True):
        to_ret = []
        to_ret.append(parent_property)
        uri = self.lightstring2uri(parent_property)
        for subj in self._ontology.subjects(RDFS.subPropertyOf, uri):
            if recursive:
                to_ret += self.getSubRelations(self.uri2lightstring(subj), True)
            else:
                to_ret.append(self.uri2lightstring(subj))
        return to_ret