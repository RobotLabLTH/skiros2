import skiros2_common.tools.logger as log
import ast


class Property(object):
    """
    @brief Touple key-value with datatype check

    If input doesn't correspond to the defined data type, it is refused

    Data type is set during initialization
    """
    __slots__ = ['_key', '_values', '_data_type']

    def __init__(self, key, value):
        """
        Value can be any value or list of values

        Value can also be a type, in such a case the _data_type is set and the value list is left empty
        """
        self._key = key
        if isinstance(value, list):
            self._values = value
            self._data_type = type(value[0])
            if not self._isListOfType(value, self._data_type):
                raise ValueError('all values must be of same type')
        elif isinstance(value, type):
            self._values = list()
            self._data_type = value
        else:
            self._values = [value]
            self._data_type = type(value)

    def isSpecified(self):
        """
        @brief Return true if the property has at least one value specified
        """
        return bool(self._values)

    def isList(self):
        """
        @brief Return true if the property has more than one specified value
        """
        return len(self._values) > 1

    @property
    def key(self):
        return self._key

    @property
    def value(self):
        return self.getValue()

    @value.setter
    def value(self, value):
        self.setValue(value)

    @property
    def values(self):
        return self.getValues()

    @values.setter
    def values(self, value):
        self.setValues(value)

    def makeInstance(self):
        """
        @brief Return an instance of the property datatype
        """
        return self._data_type()

    def dataType(self):
        """
        @brief Return the property datatype
        """
        return self._data_type

    def unset(self):
        """
        @brief Unset the property (getValue will then return None)
        """
        self._values = list()

    def dataTypeIs(self, vtype):
        """
        @brief Return true if input type match the property datatype
        """
        if isinstance(vtype, type):
            return self._data_type == vtype
        else:
            return isinstance(vtype, self._data_type)

    def setValue(self, value, index=0):
        """
        @brief Set the value at the index
        """
        if isinstance(value, self._data_type):
            if index < len(self._values):
                self._values[index] = value
            else:
                self._values.append(value)
        else:
            log.error("setValue", "{}: Input {} != {}. Debug: {}".format(self.key, type(value), self._data_type, self.printState()))
            return

    def setValueFromStr(self, value, index=0):
        """
        @brief Try to convert a string into the property datatype and set the value at index
        """
        if self.dataTypeIs(dict):
            self.setValue(ast.literal_eval(value), index)
        else:
            self.setValue(self._data_type(value), index)

    def setValuesFromStr(self, values):
        """
        @brief Try to convert a string into the property datatype
        """
        tokens = values.split(";")
        self.unset()
        for i, t in enumerate(tokens):
            self.setValueFromStr(t, i)

    def setValues(self, value):
        """
        @brief Set all the values
        """
        if value is None:
            self._values = list()
        elif isinstance(value, list):
            if self._isListOfType(value, self._data_type):
                self._values = value
            else:
                log.error("setValuesList", "{}: Input {} != {} Debug: {}. Input: {}.".format(
                    self.key, type(value[0]), self._data_type, self.printState(), value))
        elif isinstance(value, self._data_type):
            self._values = [value]
        else:
            log.error("setValues", "{}: Input {} != {}. Debug: {}".format(self.key, type(value), self._data_type, self.printState()))

    def removeValue(self, value):
        """
        @brief Removes the first value matching. Does nothing if value is not present
        """
        i = self.find(value)
        if i >= 0:
            del self._values[i]

    def find(self, value):
        """
        @brief Return the index of the value, or -1 if not found
        """
        for i, v in enumerate(self._values):
            if v == value:
                return i
        return -1

    def addValue(self, value):
        """
        @brief Append a value
        """
        if isinstance(value, self._data_type):
            self._values.append(value)
        else:
            log.error("append", self._key + ": " + str(type(value)) + "!=" + str(self._data_type))
            return

    def getValue(self, index=0):
        """
        @brief Get value at index
        """
        if not self.isSpecified():
            return None
        return self._values[index]

    def getValues(self):
        """
        @brief Get all values
        """
        return self._values

    def getValuesStr(self):
        """
        @brief Get values as string
        """
        return ";".join([str(v) for v in self._values])

    def printState(self):
        """
        @brief Return a string with key and values
        """
        v = str(self._values)
        max_lenght = 500
        return "{}:{}".format(self._key, v) if len(v) < max_lenght else "{}:{} ...]".format(self._key, v[0:max_lenght])

    def _isListOfType(self, value_list, expected_type):
        """
        @brief      Check that the elements of a list have expected type

        @param      value_list     The value list
        @param      expected_type  The expected type

        @return     True if list of type, False otherwise.
        """
        for v in value_list:
            if not isinstance(v, expected_type):
                return False
        return True





