import skiros2_common.tools.logger as log
from skiros2_common.tools.plugin_loader import PluginLoader
from skiros2_common.core.discrete_reasoner import DiscreteReasoner
from skiros2_common.core.property import Property
from datetime import datetime
import rospy

class Element(object):
    """
    @brief A touple type, label, id, properties, relation, where:
        -type reference the class type in the ontology
        -label reference a template individual in the ontology
        -id is the individual uri
        -properties is a collection of Properties
        -relations is a collection of relations with other elements

    >>> e = Element()
    >>> e.setProperty("Hello", float)
    >>> e.setProperty("Hello", 0.0)
    >>> e.getProperty("Hello").value
    0.0
    >>> e.getProperty("Hello").values = 1.0
    >>> e.getProperty("Hello").values
    [1.0]
    >>> e.getProperty("Hello").value = 2.0
    >>> e.getProperty("Hello").values
    [2.0]
    """
    __slots__ = ['_last_update', '_type', '_label', '_id', '_properties', '_local_relations','_relations']
    _plug_loader = None
    _property_reasoner_map = None

    def printState(self, verbose=False, filter=""):
        if self._id=="":
            to_ret = self._type
        else:
            to_ret = self._id
        to_ret = to_ret + "-" + self._label #+ self._properties
        if verbose:
            for k, p in self._properties.iteritems():
                if k==filter or filter=="":
                    to_ret += "\n" + p.printState()
            for r in self._relations:
                to_ret += "\n" + str(r)
        return to_ret

    def __init__(self, etype="Unknown", elabel="", eid=""):
        # Description
        self._type=etype
        self._label=elabel
        self._id=eid
        self._properties={}
        self._local_relations=[] #Reference to Elements
        self._relations=[] #Reference to IDs
        self._setLastUpdate()

    def __str__(self):
        return self.printState()


    @property
    def id(self):
        return self._id

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, t):
        self._type = t

    @property
    def label(self):
        return self._label

    @label.setter
    def label(self, l):
        self._label = l

    def available_properties(self):
        return self._properties.keys()




    def _setLastUpdate(self):
        self._last_update = datetime.now()

    def getLastUpdate(self):
        return self._last_update

    def _initPluginLoader(self):
        Element._plug_loader = PluginLoader()
        Element._property_reasoner_map = {}
        #TODO: remove dependency from ROSpy
        for package in rospy.get_param('wm/reasoners_pkgs', []):
            Element._plug_loader.load(package, DiscreteReasoner)
        for plugin in Element._plug_loader:
            r = plugin()
            Element._property_reasoner_map[r.__class__.__name__] = r
            for p in r.getAssociatedProperties():
                Element._property_reasoner_map[p] = r
            for p in r.getAssociatedRelations():
                Element._property_reasoner_map[p] = r

    def _getReasoner(self, get_code):
        """
        @brief Return the reasoner associated to get_code
        """
        if not isinstance(Element._plug_loader, PluginLoader):
            self._initPluginLoader()
        if not Element._property_reasoner_map.has_key(get_code):
            raise KeyError("No reasoner associated to data {}. Debug: {}".format(get_code, Element._property_reasoner_map))
        return Element._property_reasoner_map[get_code]

    def getIdNumber(self):
        """
        @brief Return the element id number as integer
        """
        if self._id.find('-')<0:
            return -1
        return int(self._id.split('-')[1])

    def setUri(self, eid):
        self._setLastUpdate()
        self._id = "{}-{}".format(self._type, eid)

    def hasData(self, get_code):
        """
        @brief Check if the element has data using a reasoner
        """
        return self._getReasoner(get_code).hasData(self, get_code)

    def getData(self, get_code):
        """
        @brief Extract data using a reasoner
        """
        return self._getReasoner(get_code).getData(self, get_code)


    def setData(self, set_code, data):
        """
        @brief Set data using a reasoner
        """
        self._setLastUpdate()
        self._getReasoner(set_code).addProperties(self)
        return self._getReasoner(set_code).setData(self, data, set_code)

    def getRelation(self, subj="", pred=[], obj=""):
        """
        @brief Return the first relation matching with input filters or none
        """
        to_ret = self.getRelations(subj, pred, obj)
        if to_ret:
            return to_ret[0]
        else:
            return None

    def getRelations(self, subj="", pred=[], obj=""):
        """
        @brief Return a list of all relations matching with input filters
        """
        to_ret = []
        if isinstance(pred, str):
            pred = [pred]
        if pred:
            if pred[0]=="":
                pred = []
        for r in self._relations:
            if (r['src']==subj or subj=="") and (r['type'] in pred or not pred) and (r['dst']==obj or obj==""):
                to_ret.append(r)
        return to_ret

    def addRelation(self, subj, predicate, obj, value=True):
        """
        @brief Add a relation with another element
        @subj An element or an element id
        @obj An element or an element id
        """
        self._setLastUpdate()
        if isinstance(obj, Element):
            self._local_relations.append({'src': "-1", 'type': predicate, 'dst': obj})
        elif isinstance(subj, str) and isinstance(obj, str):
            self._relations.append({'src': subj, 'type': predicate, 'dst': obj, 'state': value})

    def hasProperty(self, key, value=None):
        """
        @brief Return true if element has the property.
        @key the property to check
        @value if specified, return true if the property has that value
        """
        if value!=None and self._properties.has_key(key):
            return self.getProperty(key).find(value)!=-1
        return self._properties.has_key(key)

    def setProperty(self, key, value, datatype=None):
        """
        @brief Set the property to a value. If datatype is specified tries to convert.
        """
        self._setLastUpdate()

        if key == 'skiros:DiscreteReasoner':
            old_reasoners = self._properties[key].values:


        if datatype:
            if datatype=="xsd:double" or datatype=="xsd:float":
                self._properties[key] = Property(key, float)
                if value!=None:
                    self._properties[key].setValues(value)
            elif datatype=="xsd:int" or datatype=="xsd:integer":
                self._properties[key] = Property(key, int)
                if value!=None:
                    self._properties[key].setValues(int(value))
            elif datatype=="xsd:boolean":
                self._properties[key] = Property(key, bool)
                if value!=None:
                    self._properties[key].setValues(value)
            elif datatype=="xsd:string":
                self._properties[key] = Property(key, str)
                if value!=None:
                    self._properties[key].setValues(str(value))
            else:
                log.warn("[Element]", "Datatype {} not recognized. Set default".format(datatype))
                self._properties[key] = Property(key, value)
        else:
            if self.hasProperty(key):
                self._properties[key].setValues(value)
            else:
                self._properties[key] = Property(key, value)

        if key == 'skiros:DiscreteReasoner':
            new_reasoners = self._properties[key].values
            [self._getReasoner(v).removeProperties(self) for r in old_reasoners if r not in new_reasoners]
            [self._getReasoner(v).addProperties(self)    for r in new_reasoners if r not in old_reasoners]


    def removeProperty(self, key):
        """
        @brief Remove the property
        """
        self._setLastUpdate()

        if key == 'skiros:DiscreteReasoner':
            [self._getReasoner(v).removeProperties(self) for r in self._properties[key].values]

        del self._properties[key]


    def appendProperty(self, key, value):
        """
        @brief Append a value to the property. If property doesn't exist it is created.
        """
        self._setLastUpdate()
        if self.hasProperty(key):
            self._properties[key].append(value)
        else:
            self.setProperty(key, value)

    def getProperty(self, key):
        """
        @brief Get a property
        """
        return self._properties[key]

    def isInstance(self, abstract, wmi):
        """
        Compare the element to an abstract description
        Return true if this is a valid instance, false otherwise
        """
        #Filter by type
        if not self._type==abstract._type and not (wmi.addPrefix(self._type) in wmi.getSubClasses(abstract._type, True)):
            return False
        #Filter by label
        if not (abstract._label=="" or abstract._label=="Unknown" or self._label==abstract._label):
            return False
        #Filter by properties
        for k, p in abstract._properties.iteritems():
            if not self.hasProperty(k):
                return False
            for v in p.getValues():
                if not v in self.getProperty(k).values:
                    return False
        #Filter by relations
        for r in abstract._relations:
            if r["src"]=="-1" or r["src"]==abstract._id:#-1 is the special autoreferencial value
                #print wmi.getRelations(self._id, r["type"], -1)
                if not wmi.getRelations(self._id, r["type"], "-1"):
                    return False
            else:
                #print wmi.getRelations(-1, r["type"], self._id)
                if not wmi.getRelations("-1", r["type"], self._id):
                    return False
        return True

