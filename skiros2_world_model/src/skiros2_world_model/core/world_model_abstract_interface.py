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

class OntologyAbstractInterface:
    def lock(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def unlock(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def query_ontology(self, query, cut_prefix=False):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def set_default_prefix(self, default_prefix):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_individual(self, name):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def add_individual(self, element, ontology_name):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def add_prefix(self, parent_class):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")
    def remove_prefix(self, parent_class):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_type(self, uri):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_sub_classes(self, parent_class, recursive=False):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_sub_properties(self, parent_property, recursive=False):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

class WorldModelAbstractInterface:
    def addElement(self, e):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def updateElement(self, e):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def removeElement(self, eid):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def setRelation(self, subj_id, predicate, obj_id, value=True):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def resolveElement(self, e):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def getTemplateElement(self, individual):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def getElement(self, eid):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def getChildElements(self, e, relation_filter="", type_filter=""):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def getBranch(self, root_id, relation_filter="", type_filter=""):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def queryModel(self, subject_id=-1, predicate="", object_id=-1):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")
