from __future__ import absolute_import
from . import params
from .world_element import Element
from copy import deepcopy
import operator

operators = {'>': operator.gt,
             '<': operator.lt,
             '>=': operator.ge,
             '<=': operator.le,
             '=': operator.eq}


class ConditionBase(object):
    """
    """

    def __init__(self, clabel, subj, desired_state):
        self._desired_state = desired_state
        self._subject_key = subj
        self._label = clabel
        self._description = ""

    def __eq__(self, other):
        if self.isEqual(other):
            return True
        else:
            return False

    def __ne__(self, other):
        if self.isEqual(other):
            return False
        else:
            return True

    def remap(self, initial_key, target_key):
        if self._subject_key == initial_key:
            self._subject_key = target_key
            self._setDescription()
            #print "New description: " + self.getDescription()

    def getParamId(self, key):
        if self._params:
            return self._params.getParamValue(key)._id
        else:
            raise

    def getKeys(self):
        return [self._subject_key]

    def getDescription(self):
        return self._description

    # Virtual functions
    def _setDescription(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def evaluate(self, ph, wmi):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def setTrue(self, ph, wmi):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def revert(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def setDesiredState(self, ph):
        """ Used to resolve the element in the world model. """
        raise NotImplementedError("Not implemented in abstract class")

    def isEqual(self, other):
        """ Equality function. """
        raise NotImplementedError("Not implemented in abstract class")

    def toElement(self):
        """ World model representation . """
        raise NotImplementedError("Not implemented in abstract class")


class ConditionOr(ConditionBase):
    def __init__(self, desired_state):
        self._desired_state = desired_state
        self._label = "or"
        self._params = None
        self._children = list()
        self._setDescription()

    def addCondition(self, condition):
        self._children.append(condition)
        self._setDescription()

    def remap(self, initial_key, target_key):
        for c in self._children:
            c.remap(initial_key, target_key)
        self._setDescription()

    def getKeys(self):
        keys = list()
        for c in self._children:
            keys += c.getKeys()
        return keys

    def isEqual(self, other):
        if isinstance(other, ConditionOr):
            raise Exception("TODO")
            return True
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionOr):
            if self._desired_state != other._desired_state:
                raise Exception("TODO")
                return True
        return False

    def _setDescription(self):
        self._description = "[{}] ( ".format(self._label)
        for c in self._children:
            self._description += " {} ".format(c.getDescription())
        self._description += ")"

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        for c in self._children:
            if c.evaluate(ph, wmi):
                return self._desired_state
        return not self._desired_state

    def setTrue(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        for c in self._children:
            if not c.setTrue(ph, wmi):
                return False
        return True

    def revert(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        for c in self._children:
            if not c.revert(ph, wmi):
                return False
        return True

    def setDesiredState(self, ph):
        for c in self._children:
            c.setDesiredState(ph)

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret


class ConditionProperty(ConditionBase):
    """
    @brief Condition over an element property

    >>> ph = params.ParamHandler()
    >>> e = Element('Type')
    >>> e.setProperty('Float', 0.0)
    >>> ph.addParam('Param1', e, params.ParamTypes.Required)
    >>> equalZero = ConditionProperty('Example', 'Float', 'Param1', '=', 0.0, True)
    >>> equalZero.evaluate(ph, None)
    True
    >>> majorZero = ConditionProperty('Example', 'Float', 'Param1', '>', 0.0, True)
    >>> majorZero.evaluate(ph, None)
    False
    >>> majorMinusOne = ConditionProperty('Example', 'Float', 'Param1', '>', -1.0, True)
    >>> majorMinusOne.evaluate(ph, None)
    True
    """

    def __init__(self, clabel, olabel, subj, operator, value, desired_state):
        self._desired_state = desired_state
        self._subject_key = subj
        self._value = value
        self._label = clabel
        self._operator = operator
        self._owl_label = olabel
        self._params = None
        self._setDescription()

    def isEqual(self, other):
        if isinstance(other, ConditionProperty):
            return self._subject_key == other._subject_key and self._operator == other._operator and self._owl_label == other._owl_label and self._value == other._value and self._desired_state == other._desired_state
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionProperty):
            if self._owl_label == other._owl_label and self._value == other._value and self._desired_state != other._desired_state:
                return self.getParamId(self._subject_key) == other.getParamId(other._subject_key) or self._subject_key == other._subject_key
        return False

    def _setDescription(self):
        self._description = "[{}] {}-{}-{}{} ({})".format(self._label, self._subject_key, self._owl_label, self._operator, self._value, self._desired_state)

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        self._description = "[{}] {}-{}-{}{} ({})".format(self._label, subj, self._owl_label, self._operator, self._value, self._desired_state)
        if self._operator == "=":
            return subj.hasProperty(self._owl_label, self._value) == self._desired_state
        else:
            if not subj.hasProperty(self._owl_label):
                return False
            return operators[self._operator](subj.getProperty(self._owl_label).value, self._value) == self._desired_state

    def setTrue(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        if subj.getIdNumber() < 0:
            return False
        self._has_cache = True
        self._cache = deepcopy(subj)
        if self._desired_state:
            if not subj.hasProperty(self._owl_label, self._value):
                subj.appendProperty(self._owl_label, self._value)
        else:
            while subj.hasProperty(self._owl_label, self._value):
                subj.removePropertyValue(self._owl_label, self._value)
        self._params.specify(self._subject_key, subj)
        self._wm.update_element(subj)
        return True

    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            #print self._description + "\n{}".format(self._cache.printState(True))
            self._params.specify(self._subject_key, self._cache)
            self._wm.update_element(self._cache)
            self._has_cache = False
            return True
        return False

    def setDesiredState(self, ph):
        e = ph.getParamValue(self._subject_key)
        if e.getIdNumber() >= 0:
            return
        if self._desired_state:
            if not e.hasProperty(self._owl_label, self._value):
                e.appendProperty(self._owl_label, self._value)
        else:
            if e.hasProperty(self._owl_label, self._value):
                e.removePropertyValue(self._owl_label, self._value)

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:appliedOnType", self._owl_label)
        to_ret.setProperty("skiros:operator", self._operator)
        to_ret.setProperty("skiros:desiredValue", self._value)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret


class ConditionRelation(ConditionBase):
    def __init__(self, clabel, olabel, subj, obj, desired_state):
        self._desired_state = desired_state
        self._subject_key = subj
        self._object_key = obj
        self._label = clabel
        self._owl_label = olabel
        self._params = None
        self._setDescription()

    def remap(self, initial_key, target_key):
        if self._subject_key == initial_key:
            self._subject_key = target_key
        elif self._object_key == initial_key:
            self._object_key = target_key
        self._setDescription()

    def getKeys(self):
        return [self._subject_key, self._object_key]

    def isEqual(self, other):
        if isinstance(other, ConditionRelation):
            return self._subject_key == other._subject_key and self._owl_label == other._owl_label and self._object_key == other._object_key and self._desired_state == other._desired_state
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionRelation):
            if self._owl_label == other._owl_label and self._desired_state != other._desired_state:
                #print "{}=={} and {}=={}".format(self.getParamId(self._subject_key), other.getParamId(other._subject_key), self.getParamId(self._object_key), other.getParamId(other._object_key))
                return (self.getParamId(self._subject_key) == other.getParamId(other._subject_key) and self.getParamId(self._object_key) ==
                        other.getParamId(other._object_key)) or (self._subject_key == other._subject_key and self._object_key == other._object_key)
        return False

    def _setDescription(self):
        self._description = "[{}] {}-{}-{} ({})".format(self._label,
                                                        self._subject_key,
                                                        self._owl_label,
                                                        self._object_key,
                                                        self._desired_state)

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        obj = self._params.getParamValue(self._object_key)
        additional = ""
        if subj.getIdNumber() < 0:
            additional += ". ?x rdf:type {}".format(subj.type)
            subj = "?x"
        else:
            subj = subj.id
        if obj.getIdNumber() < 0:
            additional += ". ?y rdf:type {}".format(obj.type)
            obj = "?y"
        else:
            obj = obj.id
        if additional == "":
            v = self._wm.get_relations(subj, self._owl_label, obj)
        else:
            v = self._wm.query_ontology("SELECT * WHERE {" + "{} {} {}".format(subj, self._owl_label, obj) + additional + ".}")
        self._description = "[{}] {}({})-{}-{}({}) ({})".format(self._label, self._subject_key, subj, self._owl_label, self._object_key, obj, self._desired_state)
        #print "{} {} {} {}".format(subj, self._owl_label, obj, v)
        if v:
            return self._desired_state
        else:
            return not self._desired_state

    def setTrue(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        obj = self._params.getParamValue(self._object_key)
        if subj.getIdNumber() < 0 or obj.getIdNumber() < 0:
            return False
        #print self._description + "{} {}".format(subj.printState(), obj.printState())
        self._has_cache = True
        self._cache = self._wm.get_relations("-1", "", obj.id)
        #print self._description + "{} {}".format(subj.printState(), obj.printState())
        if not self._wm.set_relation(subj.id, self._owl_label, obj.id, self._desired_state):
            return False
        return True

    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            subj = self._params.getParamValue(self._subject_key)
            obj = self._params.getParamValue(self._object_key)
            #print self._description + "{} {}".format(subj.printState(), obj.printState())
            self._wm.set_relation(subj._id, self._owl_label, obj._id, not self._desired_state)
            for edge in self._cache:
                self._wm.set_relation(edge['src'], edge['type'], edge['dst'], True)
            self._has_cache = False
            return True
        return False

    def setDesiredState(self, ph):
        subj = ph.getParamValue(self._subject_key)
        obj = ph.getParamValue(self._object_key)
        if subj.getIdNumber() < 0:
            if not subj.hasRelation("-1", self._owl_label, self._object_key, self._desired_state):
                subj.addRelation("-1", self._owl_label, self._object_key, self._desired_state)
        elif obj.getIdNumber() < 0:
            if not obj.hasRelation("-1", self._owl_label, self._object_key, self._desired_state):
                obj.addRelation(self._subject_key, self._owl_label, "-1", self._desired_state)

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:hasObject", self._object_key)
        to_ret.setProperty("skiros:appliedOnType", self._owl_label)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret


class AbstractConditionRelation(ConditionBase):
    def __init__(self, clabel, olabel, subj, obj, desired_state):
        self._desired_state = desired_state
        self._subject_key = subj
        self._object_key = obj
        self._label = clabel
        self._owl_label = olabel
        self._params = None
        self._setDescription()

    def remap(self, initial_key, target_key):
        if self._subject_key == initial_key:
            self._subject_key = target_key
        elif self._object_key == initial_key:
            self._object_key = target_key
        self._setDescription()

    def getKeys(self):
        return [self._subject_key, self._object_key]

    def isEqual(self, other):
        if isinstance(other, ConditionRelation):
            return self._subject_key == other._subject_key and self._owl_label == other._owl_label and self._object_key == other._object_key and self._desired_state == other._desired_state
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionRelation):
            if self._owl_label == other._owl_label and self._desired_state != other._desired_state:
                #print "{}=={} and {}=={}".format(self.getParamId(self._subject_key), other.getParamId(other._subject_key), self.getParamId(self._object_key), other.getParamId(other._object_key))
                return (self.getParamId(self._subject_key) == other.getParamId(other._subject_key) and self.getParamId(self._object_key) ==
                        other.getParamId(other._object_key)) or (self._subject_key == other._subject_key and self._object_key == other._object_key)
        return False

    def _setDescription(self):
        self._description = "[{}] {}-{}-{} ({})".format(self._label,
                                                        self._subject_key,
                                                        self._owl_label,
                                                        self._object_key,
                                                        self._desired_state)

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key).type
        obj = self._params.getParamValue(self._object_key).type
        v = wmi.query_ontology("""
                               SELECT ?ytypes WHERE {{
                                       {{ ?xtypes rdfs:subClassOf* {subj}. }} UNION {{ {subj} rdfs:subClassOf* ?xtypes. }}
                                       {{ ?ytypes rdfs:subClassOf* {obj}. }} UNION {{ {obj} rdfs:subClassOf* ?ytypes. }}
                                       ?xtypes rdfs:subClassOf ?restriction . ?restriction owl:onProperty {relation}. ?restriction ?quantity ?ytypes.
                                    }}
                               """.format(subj=subj, relation=self._owl_label, obj=obj))
        #print "{} {} {}".format(subj, self._owl_label, v)
        self._description = "[{}] {}-{}-{} ({})".format(self._label, subj, self._owl_label, obj, self._desired_state)
        if v:
            return self._desired_state
        else:
            return not self._desired_state

    def setTrue(self, ph, wmi):
        return False

    def revert(self, ph, wmi):
        return False

    def setDesiredState(self, ph):
        subj = ph.getParamValue(self._subject_key)
        obj = ph.getParamValue(self._object_key)
        if subj.getIdNumber() < 0:
            if not subj.hasRelation("-1", self._owl_label, self._object_key, self._desired_state, abstract=True):
                subj.addRelation("-1", self._owl_label, self._object_key, self._desired_state, abstract=True)
        if obj.getIdNumber() < 0:
            if not obj.hasRelation("-1", self._owl_label, self._object_key, self._desired_state, abstract=True):
                obj.addRelation(self._subject_key, self._owl_label, "-1", self._desired_state, abstract=True)

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:hasObject", self._object_key)
        to_ret.setProperty("skiros:appliedOnType", self._owl_label)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret


class ConditionHasProperty(ConditionBase):
    def __init__(self, clabel, olabel, subj, desired_state):
        self._desired_state = desired_state
        self._subject_key = subj
        self._label = clabel
        self._owl_label = olabel
        self._params = None
        self._setDescription()

    def isEqual(self, other):
        if isinstance(other, ConditionHasProperty):
            return self._subject_key == other._subject_key and self._owl_label == other._owl_label and self._desired_state == other._desired_state
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionHasProperty):
            if self._owl_label == other._owl_label and self._desired_state != other._desired_state:
                return self.getParamId(self._subject_key) == other.getParamId(other._subject_key) or self._subject_key == other._subject_key
        return False

    def _setDescription(self):
        self._description = "[{}] {}-{} ({})".format(self._label,
                                                     self._subject_key,
                                                     self._owl_label,
                                                     self._desired_state)

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        #print self._description + "\n{}".format(subj.printState(True))
        if subj.getIdNumber() < 0:
            if self._params.getParam(self._subject_key).paramType == params.ParamTypes.Optional:  # If optional return true, else return false
                return True
            else:
                return False
        if subj.hasProperty(self._owl_label):
            if subj.getProperty(self._owl_label).isSpecified():
                return self._desired_state
            else:
                return not self._desired_state
        return not self._desired_state

    def setTrue(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        if subj.getIdNumber() < 0:
            return False
        self._has_cache = True
        self._cache = deepcopy(subj)
        if self._desired_state:
            if not subj.hasProperty(self._owl_label):
                subj.setProperty(self._owl_label, "")
        else:
            if subj.hasProperty(self._owl_label):
                subj.removeProperty(self._owl_label)
        self._params.specify(self._subject_key, subj)
        self._wm.update_element(subj)
        return True

    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            self._params.specify(self._subject_key, self._cache)
            #print self._description + " {}".format(self._cache.printState())
            self._wm.update_element(self._cache)
            self._has_cache = False
            return True
        return False

    def setDesiredState(self, ph):
        e = ph.getParamValue(self._subject_key)
        if e.getIdNumber() >= 0:
            return
        if self._desired_state:
            if not e.hasProperty(self._owl_label):
                e.setProperty(self._owl_label, "")
        else:
            if e.hasProperty(self._owl_label):
                e.removeProperty(self._owl_label)

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:appliedOnType", self._owl_label)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret


class ConditionIsSpecified(ConditionBase):
    def __init__(self, clabel, subj, desired_state):
        self._subject_key = subj
        self._label = clabel
        self._desired_state = desired_state
        self._params = None
        self._setDescription()

    def isEqual(self, other):
        if isinstance(other, ConditionIsSpecified):
            return self._subject_key == other._subject_key and self._desired_state == other._desired_state
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionIsSpecified):
            return self._subject_key == other._subject_key and self._desired_state != other._desired_state
        else:
            return False

    def _setDescription(self):
        self._description = "[{}] {} ({})".format(self._label,
                                                  self._subject_key,
                                                  self._desired_state)

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        if subj.getIdNumber() >= 0 and self._desired_state:
            return True
        elif subj.getIdNumber() < 0 and not self._desired_state:
            return True
        else:
            return False

    def setTrue(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        self._has_cache = True
        self._cache = deepcopy(subj)
        if subj.getIdNumber() < 0 and self._desired_state:
            self._cache_new = Element(subj._type, "==FAKE==")
            self._params.specify(self._subject_key, self._cache_new)
            self._cache_new = deepcopy(self._cache_new)
        elif subj.getIdNumber() >= 0 and not self._desired_state:
            subj._id = ""
            self._cache_new = subj
            self._params.specify(self._subject_key, self._cache_new)
        else:
            self._cache_new = self._cache
        return True

    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            #print self._description + " {}".format(self._cache.printState())
            self._params.specify(self._subject_key, self._cache)
            self._has_cache = False
            return True
        return False

    def setDesiredState(self, ph):
        return

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret


class ConditionGenerate(ConditionBase):
    # TODO: understand: any difference with isSpecified?
    def __init__(self, clabel, subj, desired_state):
        self._subject_key = subj
        self._label = clabel
        self._desired_state = desired_state
        self._params = None
        self._setDescription()

    def isEqual(self, other):
        if isinstance(other, ConditionGenerate):
            return self._subject_key == other._subject_key and self._desired_state == other._desired_state
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionGenerate):
            return self._subject_key == other._subject_key and self._desired_state != other._desired_state
        else:
            return False

    def _setDescription(self):
        self._description = "[{}] {} ({})".format(self._label,
                                                  self._subject_key,
                                                  self._desired_state)

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        if subj.getIdNumber() >= 0 and self._desired_state:
            return True
        elif subj.getIdNumber() < 0 and not self._desired_state:
            return True
        else:
            return False

    def setTrue(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        self._has_cache = True
        self._cache = deepcopy(subj)
        #print subj.printState(True)
        if subj.getIdNumber() < 0 and self._desired_state:
            self._cache_new = Element(subj._type, "==FAKE==")
            self._wm.add_element(self._cache_new, ":Scene-0", "contain")
            #print self._wm.printModel()
            self._params.specify(self._subject_key, self._cache_new)
            self._cache_new = deepcopy(self._cache_new)
        elif subj.getIdNumber() >= 0 and not self._desired_state:
            self._wm.remove_element(subj._id)
            subj._id = ""
            self._cache_new = subj
            self._params.specify(self._subject_key, self._cache_new)
        else:
            self._cache_new = self._cache
        return True

    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            #print self._description + " {}".format(self._cache_new.printState())
            if self._cache_new.getIdNumber() >= 0 and self._cache.getIdNumber() < 0:
                self._wm.remove_element(self._cache_new._id)
            elif self._cache_new.getIdNumber() < 0 and self._cache.getIdNumber() >= 0:
                self._wm.add_element(self._cache, ":Scene-0", "contain")
            self._params.specify(self._subject_key, self._cache)
            self._has_cache = False
            return True
        return False

    def setDesiredState(self, ph):
        return

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret


class ConditionOnType(ConditionBase):
    def __init__(self, clabel, subj, value):
        self._subject_key = subj
        self._label = clabel
        self._value = value
        self._params = None
        self._setDescription()

    def isEqual(self, other):
        if isinstance(other, ConditionOnType):
            return self._subject_key == other._subject_key and self._value == other._value
        else:
            return False

    def hasConflict(self, other):
        if isinstance(other, ConditionOnType):
            return self._subject_key == other._subject_key
        else:
            return False

    def _setDescription(self):
        self._description = "[{}] {} is of type {}".format(self._label,
                                                           self._subject_key,
                                                           self._value)

    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        st = self._params[self._subject_key].value.type
        types = wmi.get_sub_classes(st) + [st]
        return self._value in types

    def setTrue(self, ph, wmi):
        return True

    def revert(self, ph, wmi):
        return True

    def setDesiredState(self, ph):
        e = ph.getParamValue(self._subject_key)
        if e.getIdNumber() >= 0:
            return
        else:
            e._type = self._value

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:desiredValue", self._value)
        return to_ret
