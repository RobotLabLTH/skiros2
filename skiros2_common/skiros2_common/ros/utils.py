#import sys
#import os
#sys.path.insert(0, os.path.abspath('../'))
#import rospy
import json
import skiros2_msgs.msg as msgs
import skiros2_common.core.world_element as we
import skiros2_common.core.params as param
from pydoc import locate

try:
    unicode
    def ispy2unicode(value):
        return isinstance(value, unicode)
except NameError:
    def ispy2unicode(value):
        return False

def json_load_byteified(file_handle):
    try:
        return _byteify(
            json.load(file_handle, object_hook=_byteify),
            ignore_dicts=True
        )
    except BaseException:
        return None


def json_loads_byteified(json_text):
    try:
        return _byteify(
            json.loads(json_text, object_hook=_byteify),
            ignore_dicts=True
        )
    except BaseException:
        return None


def _byteify(data, ignore_dicts=False):
    # if this is a unicode string, return its string representation
    if ispy2unicode(data):
        return data.encode('utf-8')
    # if this is a list of values, return list of byteified values
    if isinstance(data, list):
        return [_byteify(item, ignore_dicts=True) for item in data]
    # if this is a dictionary, return dictionary of byteified keys and values
    # but only if we haven't already byteified it
    if isinstance(data, dict) and not ignore_dicts:
        return {
            _byteify(key, ignore_dicts=True): _byteify(value, ignore_dicts=True)
            for key, value in data.items()
        }
    # if it's anything else, return it in its original form
    return data


complex_types_string = []
complex_types_decoders = []

complex_types = []
complex_types_names = []
complex_types_encoders = []

ctype_map = {}


class StrEncoder(json.JSONEncoder):
    def default(self, obj):
        if name in complex_types_names:
            return complex_types_encoders[complex_types_names.index(name)](self, obj)
        # Let the base class default method raise the TypeError
        return json.JSONEncoder.default(self, obj)


class ParamsEncoder(json.JSONEncoder):
    def default(self, obj):
        name = obj.__class__.__name__
        if name in complex_types_names:
            return complex_types_encoders[complex_types_names.index(name)](self, obj)
        try:
            return json.JSONEncoder.default(self, obj)
        except TypeError:
            return "NotSerializable"


class UnicodeDecoder(json.JSONDecoder):
    def default(self, obj):
        name = obj.__class__.__name__
        if name in complex_types_names:
            return complex_types_encoders[complex_types_names.index(name)](self, obj)
        # Let the base class default method raise the TypeError
        return json.JSONEncoder.default(self, obj)


def defaultDecoder(obj):
    return obj


def registerDecoder(name, func):
    if name in complex_types_string:
        print("{} already registered as decoder".format(name))
        return
    complex_types_string.append(name)
    complex_types_decoders.append(func)


def registerEncoder(ctype, func):
    class_name = ctype.__name__
    if class_name in complex_types_names:
        print("{} already registered as encoder".format(class_name))
        return
    complex_types.append(ctype)
    complex_types_names.append(class_name)
    complex_types_encoders.append(func)


def registerClass(name, class_type, encoder=json.JSONEncoder.default, decoder=defaultDecoder):
    """
    Class registration is used to translate a complex type to C element and vice-versa
    """
    registerDecoder(name, decoder)
    registerEncoder(class_type, encoder)


def registerCtype(name, class_type):
    """
    C types are used to translate a type to a C type string. To use when the C string is already used by a registered class
    """
    class_name = class_type.__name__
    if class_name in complex_types_names:
        print("{} already registered as encoder, can t be registered as a C type".format(class_name))
        return
    ctype_map[class_name] = name


def getStrFromType(obj):
    """
    Get the string given an object
    """
    name = obj.__name__
    if name in complex_types_names:
        return complex_types_string[complex_types_names.index(name)]
    elif name in ctype_map:
        return ctype_map[name]
    else:
        return name


def getTypeFromStr(name):
    """
    Get an object given a string
    """
    if name in complex_types_string:
        return complex_types[complex_types_string.index(name)]()
    else:
        return locate(name)()


def encodeParam(encoder, obj):
    return {"key": obj._key, "description": obj._description, "specType": obj._param_type.value - 1, "type": getStrFromType(obj._data_type),
            "values": obj._values}


def decodeParam(p):
    if isinstance(p, str):
        v = json_loads_byteified(p)
    else:
        v = p
    if len(v['values']) > 0:
        return param.Param(v['key'], v['description'], decode(v['values'], v['type']), v['specType'])
    else:
        return param.Param(v['key'], v['description'], getTypeFromStr(v['type']).__class__, v['specType'])


def encodeProperty(encoder, obj):
    return {"key": obj._key, "type": getStrFromType(obj._data_type), "values": obj._values}


def decodeProperty(p):
    if isinstance(p, str):
        v = json_loads_byteified(p)
    else:
        v = p
    if len(v['values']) > 0:
        return param.Property(v['key'], decode(v['values'], v['type']))
    else:
        return param.Property(v['key'], getTypeFromStr(v['type']).__class__)


def encodeElement(encoder, obj):
    return {"id": obj._id, "label": obj._label, "type": obj._type, "last_update": 0.0,
            "properties": {k: encoder.default(v) for k, v in obj._properties.items()}, "relations": obj._relations}


def decodeElement(json_string):
    # TODO json_string['last_update']
    e = we.Element(json_string['type'], json_string['label'], json_string['id'])
    for k, p in json_string['properties'].items():
        e._properties[k] = decodeProperty(p)
    e._relations = json_string['relations']
    return e


registerClass("skiros_wm::Element", we.Element, encodeElement, decodeElement)
registerClass("skiros_wm::Param", param.Param, encodeParam)
registerClass("skiros_wm::Property", param.Property, encodeProperty)
#registerClass("std::string", unicode)
#registerCtype("std::string", str)
#registerClass("double", float)


def decode(values, data_type):
    if data_type in complex_types_string:
        return [complex_types_decoders[complex_types_string.index(data_type)](v) for v in values]
    else:
        return values


def serializeParamMap(param_map):
    """
    >>> ph = param.ParamHandler()
    >>> ph.addParam("MyDict", dict, param.ParamTypes.Required)
    >>> serializeParamMap(ph._params)
    [param: "{\\"values\\": [], \\"specType\\": 0, \\"type\\": \\"dict\\", \\"description\\": \\"\\", \\"key\\"\\
      : \\"MyDict\\"}"]
    >>> ph.addParam("MyList", list, param.ParamTypes.Required)
    >>> serializeParamMap(ph._params)
    [param: "{\\"values\\": [], \\"specType\\": 0, \\"type\\": \\"list\\", \\"description\\": \\"\\", \\"key\\"\\
      : \\"MyList\\"}", param: "{\\"values\\": [], \\"specType\\": 0, \\"type\\": \\"dict\\", \\"description\\": \\"\\", \\"key\\"\\
      : \\"MyDict\\"}"]
    >>> params = {}
    >>> params["MyDict"] = param.Param("MyDict", "", dict, param.ParamTypes.Required)
    >>> serializeParamMap(params)
    [param: "{\\"values\\": [], \\"specType\\": 0, \\"type\\": \\"dict\\", \\"description\\": \\"\\", \\"key\\"\\
      : \\"MyDict\\"}"]
    >>> params = {}
    >>> params["MyString"] = param.Param("MyString", "", "String", param.ParamTypes.Required)
    >>> serializeParamMap(params)
    [param: "{\\"values\\": [\\"String\\"], \\"specType\\": 0, \\"type\\": \\"str\\", \\"description\\": \\"\\
      \\", \\"key\\": \\"MyString\\"}"]
    """
    s_param_map = []
    for p in param_map.values():
        msg = msgs.Param()
        msg.param = str(json.dumps(p, cls=ParamsEncoder))
        s_param_map.append(msg)
    return s_param_map


def deserializeParamMap(params):
    """
    >>> ph = param.ParamHandler()
    >>> ph.addParam("MyDict", dict, param.ParamTypes.Required)
    >>> param.ParamHandler(deserializeParamMap(serializeParamMap(ph._params))).printState()
    'MyDict:[] '
    >>> ph.addParam("MyList", list, param.ParamTypes.Required)
    >>> param.ParamHandler(deserializeParamMap(serializeParamMap(ph._params))).printState()
    'MyDict:[] MyList:[] '
    """
    param_map = {}
    for p in params:
        dp = decodeParam(p.param)
        if dp is not None:
            param_map[dp._key] = dp
    return param_map


def deserializePropertyMap(msg):
    """
    >>> params = {}
    >>> params["MyDict"] = param.Property("MyDict", dict)
    >>> params["MyFloat"] = param.Property("MyFloat", float)
    >>> params = deserializePropertyMap(serializePropertyMap(params))
    >>> for p in params.values(): p.printState()
    'MyDict:[]'
    'MyFloat:[]'
    >>> params["MyFloat"].value = 1.0
    >>> for p in params.values(): p.printState()
    'MyDict:[]'
    'MyFloat:[1.0]'
    """
    p_map = {}
    for p in msg:
        data_value = json_loads_byteified(p.data_value)
        if len(data_value) > 0:
            p_map[p.key] = param.Property(p.key, decode(data_value, p.data_type))
        else:
            p_map[p.key] = param.Property(p.key, getTypeFromStr(p.data_type).__class__)
    return p_map


def serializePropertyMap(p_map):
    """
    >>> params = {}
    >>> params["MyDict"] = param.Property("MyDict", dict)
    >>> serializePropertyMap(params)
    [key: "MyDict"
    data_value: "[]"
    dataType: "dict"]
    >>> params = {}
    >>> params["MyString"] = param.Property("MyString", "String")
    >>> serializePropertyMap(params)
    [key: "MyString"
    data_value: "[\\"String\\"]"
    dataType: "str"]
    """
    s_p_map = []
    for p in p_map.values():
        msg = msgs.Property()
        msg.key = p.key
        msg.data_value = str(json.dumps(p._values, cls=ParamsEncoder))
        msg.data_type = getStrFromType(p._data_type)
        s_p_map.append(msg)
    return s_p_map


def msg2element(msg):
    e = we.Element()
    e._id = msg.id
    e._label = msg.label
    e._type = msg.type
    e._properties = deserializePropertyMap(msg.properties)
    for r in msg.relations:
        e.addRelation(r.subject_id, r.predicate, r.object_id)
    return e


def element2msg(element):
    msg = msgs.WmElement()
    msg.id = element.id
    msg.label = element.label
    msg.type = element.type
    msg.properties = serializePropertyMap(element._properties)
    for r in element._relations:
        msg.relations.append(relation2msg(r))
    return msg


def relation2msg(r):
    rmsg = msgs.Relation()
    rmsg.subject_id = r['src']
    rmsg.predicate = r['type']
    rmsg.object_id = r['dst']
    return rmsg


def msg2relation(msg):
    return makeRelation(msg.subject_id, msg.predicate, msg.object_id)


def makeRelation(subj, pred, obj):
    return {'src': subj, 'type': pred, 'dst': obj}


def makeRelationMsg(subj, pred, obj):
    return msgs.Relation(subject_id=subj, predicate=pred, object_id=obj)


def makeStatementMsg(subj, pred, obj, value):
    rmsg = msgs.Statement()
    rmsg.relation = makeRelationMsg(subj, pred, obj)
    rmsg.value = value
    return rmsg
