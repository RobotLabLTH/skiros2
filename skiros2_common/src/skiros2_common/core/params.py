from flufl.enum import Enum
from copy import copy, deepcopy
import skiros2_common.tools.logger as log
from skiros2_common.core.property import Property
from skiros2_common.core.world_element import Element
from datetime import datetime, timedelta

"""
Required: parameters that must be specified
Optional: optional parameters
Inferred: a required parameter that is auto-parameterized by pre-conditions
"""
ParamTypes = Enum('ParamTypes', 'Required Optional Inferred')


class Param(Property):
    """
    @brief A param is a property with addionally:
        *a default value and a parameter type
        *functions to handle World Elements and possibility to be converted in a World Element

    >>> p = Param("MyProp", "", 0, ParamTypes.Required)
    >>> p.value
    0
    >>> t = p.last_update
    >>> p.hasChanges(t)
    False
    >>> p.value = 1
    >>> p.hasChanges(t)
    True

    """
    __slots__ = ['_key', '_description', '_param_type', '_data_type', '_is_list', '_values', '_default', '_last_update']

    def __init__(self, key, description, value, param_type, is_list=False):
        self._is_list = is_list
        self._key = key
        self._description = description
        self._setLastUpdate()
        if isinstance(param_type, int):
            self._param_type = ParamTypes(param_type + 1)
        else:
            self._param_type = param_type
        if isinstance(value, list):
            self._values = value
            self._data_type = type(value[0])
        elif isinstance(value, type):
            self._data_type = value
            self._values = list()
        else:
            self._data_type = type(value)
            self._values = [value]
        self._default = deepcopy(self._values)

    def __copy__(self):
        result = self.__class__.__new__(self.__class__)
        for k in self.__slots__:
            setattr(result, k, getattr(self, k))
        return result

    def __deepcopy__(self, memo):
        result = self.__copy__()
        memo[id(self)] = result
        return result

    @property
    def default(self):
        return self.getDefaultValue()

    @property
    def defaults(self):
        return self.getDefaultValues()

    @property
    def last_update(self):
        return self._last_update

    def _setLastUpdate(self):
        """
        @brief Update the time of last update
        """
        self._last_update = datetime.now()

    def hasChanges(self, time, tolerance=1):
        """
        @brief Returns true if the property was changes since the specified time
        @time a datetime
        @tolerance the tolerance in microseconds
        """
        if (self._last_update - time) > timedelta(microseconds=tolerance):
            return True
        return False

    def setValue(self, value, index=0):
        """
        @brief Set the value at the index
        """
        super(Param, self).setValue(value, index)
        self._setLastUpdate()

    def setValues(self, value):
        """
        @brief Set all the values
        """
        super(Param, self).setValues(value)
        self._setLastUpdate()

    def removeValue(self, value):
        """
        @brief Removes the first value matching. Does nothing if value is not present
        """
        super(Param, self).removeValue(value)
        self._setLastUpdate()

    def addValue(self, value):
        """
        @brief Append a value
        """
        super(Param, self).addValue(value)
        self._setLastUpdate()

    def hasSpecifiedDefault(self):
        """
        @brief Check if the current parameter has default values specified
        """
        return bool(self._default)

    def getDefaultValue(self, index=0):
        if not self._default:
            return None
        return self._default[index]

    def getDefaultValues(self):
        return self._default

    def hasDefaultValues(self):
        """
        @brief Check if the current parameter value is the default
        """
        for v1, v2 in zip(self._default, self._values):
            if v1 != v2:
                return False
        return True

    def makeDefault(self, values):
        """
        @brief Specify the parameter and set it as default value
        """
        self.setValues(values)
        self._default = deepcopy(self._values)

    def setDefault(self):
        """
        @brief Set the parameter to default value
        """
        self._values = deepcopy(self._default)

    @property
    def description(self):
        return self._description

    @property
    def paramType(self):
        return self._param_type

    def paramTypeIs(self, ptype):
        return self._param_type == ptype

    def toElement(self):
        to_ret = Element("skiros:Parameter", self._key)
        to_ret.setProperty("rdfs:comment", self._description)
        to_ret.setProperty("skiros:ParameterType", int(self._param_type) - 1)
        if(not self.dataTypeIs(Element)):
            to_ret.setProperty("skiros:DataType", self._data_type)
            if self.hasSpecifiedDefault():
                to_ret.setProperty("skiros:Default", self._default)
            if self.isSpecified():
                to_ret.setProperty("skiros:Value", self._values)
        else:
            to_ret.setProperty("skiros:DataType", self._default[0]._type)
            if self.isSpecified():
                for v in self._values:
                    if v._id != "":
                        to_ret.addRelation("-1", "skiros:hasValue", v._id)
        return to_ret


class ParamHandler(object):
    """
    >>> ph = ParamHandler()
    >>> ph.addParam("Trajectory", dict, ParamTypes.Required)
    >>> ph.printState()
    'Trajectory:[] '
    >>> ph.specify("Trajectory", {"MyTraj": "Ue"})
    >>> ph.printState()
    "Trajectory:[{'MyTraj': 'Ue'}] "
    >>> ph.setDefault("Trajectory")
    >>> ph.printState()
    'Trajectory:[] '

    """
    __slots__ = ['_params']

    def __init__(self, params=None):
        self._params = {}
        if params:
            self._params = params

    def __getitem__(self, key):
        if self.hasParam(key):
            return self._params[key]
        else:
            log.error('ParamHandler', 'Param {} is not in the map. Debug: {}'.format(key, self.printState()))

    def keys(self):
        return self._params.keys()

    def values(self):
        return self._params.values()

    def iteritems(self):
        return self._params.iteritems()

    def reset(self, copy):
        self._params = copy

    def getCopy(self):
        return deepcopy(self._params)

    def getParamMap(self):
        return self._params

    def merge(self, other):
        """
        @brief Return the parameter map, result of the merge between self and another ParameterHandler
        """
        to_ret = self.getCopy()
        for key, param in other._params.iteritems():
            if self.hasParam(key):
                if not param.isSpecified():  # If not specified (None) parameter is removed
                    del to_ret[key]
                else:
                    to_ret[key].setValues(param.getValues())
            else:
                if param.isSpecified():  # Only if specified (not None) parameter is added
                    to_ret[key] = param
        return to_ret

    def remap(self, initial_key, target_key):
        """
        @brief Remap a parameter to a new key
        """
        if self.hasParam(initial_key):
            #print self.printState()
            temp = self._params[initial_key]
            temp._key = target_key
            self._params[target_key] = temp
            del self._params[initial_key]
            #print 'after ' + self.printState()

    def specifyParams(self, other, keep_default=False):
        """
        @brief Set the input params
        """
        for key, param in other._params.iteritems():
            if self.hasParam(key):
                t = self._params[key]
                if keep_default and t.hasSpecifiedDefault():
                    if t.dataTypeIs(Element):
                        if t.getDefaultValue().getIdNumber() < 0 or set(v.getIdNumber() for v in t.getDefaultValues()) == set(v.getIdNumber() for v in param.values):
                            t.values = param.values
                else:
                    t.values = param.values

    def specifyParamsDefault(self, other):
        """
        @brief Set the input params and default value
        """
        for key, param in other._params.iteritems():
            if self.hasParam(key):
                self._params[key].makeDefault(param.getValues())

    def hasParam(self, key):
        """
        @brief Check that a key exists and return false otherwise
        """
        return key in self._params

    def setDefault(self, key=None):
        """
        @brief Set the param (or the params, if key is a list) to the default value
        """
        if isinstance(key, list):
            for k in key:
                self._params[k].setDefault()
        elif key is None:
            for _, p in self._params.iteritems():
                p.setDefault()
        else:
            self._params[key].setDefault()

    def addParam(self, key, value, param_type, description=""):
        self._params[key] = Param(key, description, value, param_type)

    def getParam(self, key):
        if self.hasParam(key):
            return self._params[key]
        else:
            log.error('getParam', 'Param {} is not in the map. Debug: {}'.format(key, self.printState()))

    def specifyDefault(self, key, values):
        if self.hasParam(key):
            self._params[key].makeDefault(values)
        else:
            log.error('specifyDefault', 'Param {} is not in the map. Debug: {}'.format(key, self.printState()))

    def specify(self, key, values):
        if self.hasParam(key):
            self._params[key].setValues(values)
        else:
            log.error('specify', 'Param {} is not in the map. Debug: {}'.format(key, self.printState()))

    def isSpecified(self, key):
        return self._params[key].isSpecified()

    def getParamValue(self, key, make_instance=False):
        """
        Return the first value of the parameter

        If make_instance is True and the parameter is not specified, an instance is returned
        rather than None
        """
        if self.hasParam(key):
            if make_instance and not self.isSpecified(key):
                return self._params[key].makeInstance()
            return self._params[key].getValue()
        else:
            log.error('getParamValue', 'Param {} is not in the map. Debug: {}'.format(key, self.printState()))

    def getParamValues(self, key):
        """
        Return the parameter values (list)

        If make_instance is True and the parameter is not specified, an instance list is returned
        rather than a None list
        """
        if self.hasParam(key):
            return self._params[key].getValues()
        else:
            log.error('getParamValues', 'Param {} is not in the map. Debug: {}'.format(key, self.printState()))

    def getElementParams(self):
        to_ret = {}
        for key, param in self._params.iteritems():
            if isinstance(param.value, Element):
                to_ret[key] = param
        return to_ret

    def getParamMapFiltered(self, type_filter):
        to_ret = {}
        for key, param in self._params.iteritems():
            if isinstance(type_filter, list):
                if param.paramType in type_filter:
                    to_ret[key] = param
            else:
                if param.paramType == type_filter:
                    to_ret[key] = param
        return to_ret

    def printState(self):
        to_ret = ""
        for _, p in self._params.iteritems():
            if not p.dataTypeIs(Element) or not p.isSpecified():
                to_ret += p.printState() + " "
            else:
                to_ret += p.key + ": " + p.value.printState() + " "
        return to_ret
