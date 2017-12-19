import params
from world_element import Element
from copy import deepcopy

class ConditionBase(object):
    """
    """    
    def __init__(self, clabel, subj, desired_state):            
        self._desired_state=desired_state                           
        self._subject_key=subj  
        self._label=clabel  
        self._description=""        
          
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
        if self._subject_key==initial_key:
            self._subject_key=target_key
            self._setDescription()
            #print "New description: " + self.getDescription()
          
    def getParamIndex(self, key):
        l = [i for i, x in enumerate(self.getKeys()) if key==x]
        if l:
            return l[0]
        else:
            return -1
    
    def getParamId(self, key):
        if self._params:
            return self._params.getParamValue(key)._id
        else:
            raise
    
    def getKeys(self):
        return [self._subject_key]
        
    def getDescription(self):  
        return self._description        
        
    #Virtual functions
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
    
    
class ConditionProperty(ConditionBase):
    def __init__(self, clabel, olabel, subj, operator, value, desired_state):            
        self._desired_state=desired_state                           
        self._subject_key=subj          
        self._value=value 
        self._label=clabel    
        self._operator=operator           
        self._owl_label=olabel
        self._params = None
        self._setDescription()
                 
    def isEqual(self, other):
        if isinstance(other, ConditionProperty):
            return self._subject_key==other._subject_key and self._owl_label==other._owl_label and self._value==other._value and self._desired_state==other._desired_state 
        else:
            return False 
            
    def hasConflict(self, other):
        if isinstance(other, ConditionProperty):
            if self._owl_label==other._owl_label and self._value==other._value and self._desired_state!=other._desired_state:
                return self.getParamId(self._subject_key)==other.getParamId(other._subject_key) or self._subject_key==other._subject_key
        return False 
            
    def _setDescription(self):  
        self._description = "[{}] {}-{}-{} ({})".format(self._label, 
                              self._subject_key, 
                              self._owl_label, 
                              self._value, 
                              self._desired_state) 
        
    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        if subj.getIdNumber() < 0:
            if self._params.getParam(self._subject_key).paramType==params.ParamTypes.Optional:#If optional return true, else return false
                return True
            else:
                return False
        if subj.hasPropertyValue(self._owl_label, self._value):
            return self._desired_state
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
            if not subj.hasPropertyValue(self._owl_label, self._value):
                subj.appendProperty(self._owl_label, self._value)
        else:
            while subj.hasPropertyValue(self._owl_label, self._value):
                subj.removePropertyValue(self._owl_label, self._value)
        self._params.specify(self._subject_key, subj)
        self._wm.updateElement(subj)
        return True
        
    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            #print self._description + "\n{}".format(self._cache.printState(True))
            self._params.specify(self._subject_key, self._cache)
            self._wm.updateElement(self._cache)
            self._has_cache = False
            return True            
        return False
        
    def setDesiredState(self, ph):
        e = ph.getParamValue(self._subject_key)
        if e.getIdNumber()>=0: return
        if self._desired_state:
            if not e.hasPropertyValue(self._owl_label, self._value):
                e.appendProperty(self._owl_label, self._value)
        else:
            if e.hasPropertyValue(self._owl_label, self._value):
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
        self._desired_state=desired_state                           
        self._subject_key=subj                                   
        self._object_key=obj      
        self._label=clabel          
        self._owl_label=olabel
        self._params = None
        self._setDescription()
        
    def remap(self, initial_key, target_key):
        if self._subject_key==initial_key:
            self._subject_key=target_key
        elif self._object_key==initial_key:
            self._object_key=target_key
        self._setDescription()
            
    def getKeys(self):
        return [self._subject_key, self._object_key]
                         
    def isEqual(self, other):
        if isinstance(other, ConditionRelation):
            return self._subject_key==other._subject_key and self._owl_label==other._owl_label and self._object_key==other._object_key and self._desired_state==other._desired_state 
        else:
            return False
            
    def hasConflict(self, other):
        if isinstance(other, ConditionRelation):
            if self._owl_label==other._owl_label and self._desired_state!=other._desired_state:
                #print "{}=={} and {}=={}".format(self.getParamId(self._subject_key), other.getParamId(other._subject_key), self.getParamId(self._object_key), other.getParamId(other._object_key))
                return (self.getParamId(self._subject_key)==other.getParamId(other._subject_key) and self.getParamId(self._object_key)==other.getParamId(other._object_key)) or (self._subject_key==other._subject_key and self._object_key==other._object_key)
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
        #print self._description + "{} {}".format(subj.printState(), obj.printState())
        if subj.getIdNumber() < 0 or obj.getIdNumber() < 0:
            ps = self._params.getParam(self._subject_key)
            po = self._params.getParam(self._object_key)
            if ((subj.getIdNumber() < 0 and ps.paramType==params.ParamTypes.Optional) or (obj.getIdNumber() < 0 and po.paramType==params.ParamTypes.Optional)):
                return True
            else:
                return False
        v = self._wm.getRelations(subj._id, self._owl_label, obj._id)
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
        self._cache = self._wm.getRelations("-1", "", obj._id)
        #print self._description + "{} {}".format(subj.printState(), obj.printState())
        if not self._wm.setRelation(subj._id, self._owl_label, obj._id, self._desired_state):
            return False
        return True
        
    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            subj = self._params.getParamValue(self._subject_key)
            obj = self._params.getParamValue(self._object_key)
            #print self._description + "{} {}".format(subj.printState(), obj.printState())
            self._wm.setRelation(subj._id, self._owl_label, obj._id, not self._desired_state)
            for edge in self._cache:
                self._wm.setRelation(edge['src'], edge['type'], edge['dst'], True)
            self._has_cache = False      
            return True
        return False
        
    def setDesiredState(self, ph):
        subj = ph.getParamValue(self._subject_key)
        obj = ph.getParamValue(self._object_key)
        if subj.getIdNumber()<0:
            subj.addRelation("-1", self._owl_label, self._object_key, self._desired_state)
        if obj.getIdNumber()<0:
            obj.addRelation(self._subject_key, self._owl_label, "-1", self._desired_state)
            
    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:hasObject", self._object_key)
        to_ret.setProperty("skiros:appliedOnType", self._owl_label)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret

           
class AbsConditionRelation(ConditionBase):
    def __init__(self, clabel, olabel, subj, obj, desired_state):  
        self._desired_state=desired_state                           
        self._subject_key=subj                                   
        self._object_key=obj      
        self._label=clabel          
        self._owl_label=olabel
        self._params = None
        self._setDescription()
        
    def remap(self, initial_key, target_key):
        if self._subject_key==initial_key:
            self._subject_key=target_key
        elif self._object_key==initial_key:
            self._object_key=target_key
        self._setDescription()
            
    def getKeys(self):
        return [self._subject_key, self._object_key]
                         
    def isEqual(self, other):
        if isinstance(other, ConditionRelation):
            return self._subject_key==other._subject_key and self._owl_label==other._owl_label and self._object_key==other._object_key and self._desired_state==other._desired_state 
        else:
            return False
            
    def hasConflict(self, other):
        if isinstance(other, ConditionRelation):
            if self._owl_label==other._owl_label and self._desired_state!=other._desired_state:
                #print "{}=={} and {}=={}".format(self.getParamId(self._subject_key), other.getParamId(other._subject_key), self.getParamId(self._object_key), other.getParamId(other._object_key))
                return (self.getParamId(self._subject_key)==other.getParamId(other._subject_key) and self.getParamId(self._object_key)==other.getParamId(other._object_key)) or (self._subject_key==other._subject_key and self._object_key==other._object_key)
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
        if not subj.hasProperty("skiros:Template") or not obj.hasProperty("skiros:Template"):
            return self._desired_state
        subj = subj.getProperty("skiros:Template").value
        obj = obj.getProperty("skiros:Template").value
        v = self._wm.getTriples(subj, self._owl_label)
        if obj in v:
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
        if subj.getIdNumber()<0:
            subj.addRelation("-1", self._owl_label, self._object_key, self._desired_state)
        if obj.getIdNumber()<0:
            obj.addRelation(self._subject_key, self._owl_label, "-1", self._desired_state)
            
    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        to_ret.setProperty("skiros:hasObject", self._object_key)
        to_ret.setProperty("skiros:appliedOnType", self._owl_label)
        to_ret.setProperty("skiros:desiredState", self._desired_state)
        return to_ret

class ConditionHasProperty(ConditionBase):
    def __init__(self, clabel, olabel, subj, desired_state):            
        self._desired_state=desired_state                           
        self._subject_key=subj      
        self._label=clabel          
        self._owl_label=olabel
        self._params = None
        self._setDescription()

    def isEqual(self, other):
        if isinstance(other, ConditionHasProperty):
            return self._subject_key==other._subject_key and self._owl_label==other._owl_label and self._desired_state==other._desired_state 
        else:
            return False
            
    def hasConflict(self, other):
        if isinstance(other, ConditionHasProperty):
            if self._owl_label==other._owl_label and self._desired_state!=other._desired_state:
                return self.getParamId(self._subject_key)==other.getParamId(other._subject_key) or self._subject_key==other._subject_key
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
            if self._params.getParam(self._subject_key).paramType==params.ParamTypes.Optional:#If optional return true, else return false
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
        self._wm.updateElement(subj)
        return True
        
    def revert(self, ph, wmi):
        if self._has_cache:
            self._params = ph
            self._wm = wmi
            self._params.specify(self._subject_key, self._cache)
            #print self._description + " {}".format(self._cache.printState())
            self._wm.updateElement(self._cache)
            self._has_cache = False
            return True            
        return False
        
    def setDesiredState(self, ph):
        e = ph.getParamValue(self._subject_key)
        if e.getIdNumber()>=0: return
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
        self._subject_key=subj      
        self._label=clabel   
        self._desired_state=desired_state  
        self._params = None 
        self._setDescription()
              
    def isEqual(self, other):
        if isinstance(other, ConditionIsSpecified):
            return self._subject_key==other._subject_key and self._desired_state==other._desired_state 
        else:
            return False
            
    def hasConflict(self, other):
        if isinstance(other, ConditionIsSpecified):
            return self._subject_key==other._subject_key and self._desired_state!=other._desired_state 
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
    #TODO: understand: any difference with isSpecified?
    def __init__(self, clabel, subj, desired_state):
        self._subject_key=subj      
        self._label=clabel   
        self._desired_state=desired_state  
        self._params = None 
        self._setDescription()
               
    def isEqual(self, other):
        if isinstance(other, ConditionGenerate):
            return self._subject_key==other._subject_key and self._desired_state==other._desired_state 
        else:
            return False
             
    def hasConflict(self, other):
        if isinstance(other, ConditionGenerate):
            return self._subject_key==other._subject_key and self._desired_state!=other._desired_state 
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
            self._wm.addElement(self._cache_new, ":Scene-0", "contain")
            #print self._wm.printModel()
            self._params.specify(self._subject_key, self._cache_new)
            self._cache_new = deepcopy(self._cache_new)
        elif subj.getIdNumber() >= 0 and not self._desired_state:
            self._wm.removeElement(subj._id)
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
            if self._cache_new.getIdNumber()>=0 and self._cache.getIdNumber()<0:
                self._wm.removeElement(self._cache_new._id)
            elif self._cache_new.getIdNumber()<0 and self._cache.getIdNumber()>=0:
                self._wm.addElement(self._cache, ":Scene-0", "contain")
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
        self._subject_key=subj      
        self._label=clabel   
        self._value=value  
        self._params = None 
        self._setDescription()
               
    def isEqual(self, other):
        if isinstance(other, ConditionOnType):
            return self._subject_key==other._subject_key and self._value==other._value 
        else:
            return False
             
    def hasConflict(self, other):
        if isinstance(other, ConditionOnType):
            return self._subject_key==other._subject_key
        else:
            return False
            
    def _setDescription(self):  
        self._description = "[{}] {}-{}".format(self._label, 
                              self._subject_key, 
                              self._value) 
                                  
    def evaluate(self, ph, wmi):
        self._params = ph
        self._wm = wmi
        subj = self._params.getParamValue(self._subject_key)
        if wmi.isOfType(subj, self._value):
            return True
        else:
            return False
        
    def setTrue(self, ph, wmi):   
        return True
        
    def revert(self, ph, wmi):       
        return True
        
    def setDesiredState(self, ph):
        e = ph.getParamValue(self._subject_key)
        if e.getIdNumber()>=0: 
            return
        else: 
            e._type = self._value

    def toElement(self):
        to_ret = Element("skiros:" + self.__class__.__name__, self._label)
        to_ret.setProperty("skiros:hasSubject", self._subject_key)
        return to_ret