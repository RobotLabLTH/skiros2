import unittest
import skiros2_common.core.params as params
import skiros2_common.core.world_element as we

def paramMapValues(ph):
    # map paramhandler keys to a list of their parameter values 
    return {k: ph.getParam(k).values for k in ph.keys()} 

class TestParamHandler(unittest.TestCase):

    def setUp(self):
        self.longMessage = True
        self.ph1 = params.ParamHandler()
        self.ph2 = params.ParamHandler()
        self.ph1.addParam("a", 1, params.ParamTypes.Required)
        self.ph2.addParam("b", 2, params.ParamTypes.Required)
        msg = """
        addValue should add the value and key to the property"""
        self.assertEqual({"a": [1]}, paramMapValues(self.ph1), msg)

    def _toValues(self, param_dict):
        # map Param keys in param_dict to a list of their values 
        return {k: v.values for k, v in param_dict.items()} 
    
    def _mergeAndMapValues(self, ph1, ph2):
        param_dict = ph1.merge(ph2)
        return self._toValues(param_dict) 
        
    def test_merge(self):
        value_dict = self._mergeAndMapValues(self.ph1, self.ph2)
        msg = """
        merge should map paramters in self and other when both are
        specified"""
        self.assertEqual({"a": [1], "b": [2]}, value_dict, msg)
        
        msg = """
        merge should override parameter in self with parameter in 
        other if they have the same key"""
        self.ph1.addParam("c", 1, params.ParamTypes.Required)
        self.ph2.addParam("c", 3, params.ParamTypes.Required)
        value_dict = self._mergeAndMapValues(self.ph1, self.ph2)
        self.assertEqual({"a": [1], "b": [2], "c": [3]}, value_dict, msg)

        msg = """
        merge should remove mapping if parameter is not specified in
        other ParamHandler"""
        self.ph1.addParam("b", 1, params.ParamTypes.Required)
        self.ph2.addParam("a", float, params.ParamTypes.Required)
        value_dict = self._mergeAndMapValues(self.ph1, self.ph2) 
        self.assertEqual({"b": [2], "c": [3]}, value_dict, msg)
 
    def test_specify(self):
        ph = self.ph1
        # overrides the param given to "a" from the setup
        ph.addParam("a", dict, params.ParamTypes.Required)
        ph.specify("b", {"c": 1})
        msg = """
        specify should have no effect if key is not in ParamHandler"""
        self.assertEqual({"a": []}, paramMapValues(ph), msg)
        
        msg = """
        specify should set the value of the parameter for the given
        key"""
        ph.specify("a", {"MyTraj": "Ue"})
        self.assertEqual({"a": [{"MyTraj": "Ue"}]}, paramMapValues(ph), msg)

    def test_remap(self):
        ph = self.ph1
        ph.remap("a", "b")
        msg = """
        remap should remap the key for a parameter to a new key
        """
        self.assertEqual({"b": [1]}, paramMapValues(ph), msg)

        msg = """
        remap should have no effect when initial_key and target_key 
        are the same"""
        ph.remap("b", "b")
        self.assertEqual({"b": [1]}, paramMapValues(ph), msg)


    def test_specifyParams(self):
        ph1, ph2 = self.ph1, self.ph2
        ph1.specifyParams(ph2)
        msg = """
        specifyParams should have no effect if there are no common
        keys between the two ParamHandlers"""
        self.assertEqual({"a": [1]}, paramMapValues(ph1), msg)
        
        msg = """
        specifyParams should specify parameters from other 
        ParamHandler for the keys that they share"""
        ph2.addParam("a", 2, params.ParamTypes.Required)
        ph1.specifyParams(ph2)
        self.assertEqual({"a": [2]}, paramMapValues(ph1), msg)

    def test_specifyParamsDefault(self):
        ph1, ph2 = self.ph1, self.ph2
        ph1.specifyParamsDefault(ph2)
        msg = """
        specifyParamsDefault should have no effect if there are no 
        common keys between the two ParamHandlers"""
        self.assertEqual({"a": [1]}, paramMapValues(ph1), msg)
       
        msg = """
        specifyParamsDefault should specify the parameters and set
        them as default"""
        ph2.addParam("a", 2, params.ParamTypes.Required)
        ph1.specifyParamsDefault(ph2)
        self.assertEqual({"a": [2]}, paramMapValues(ph1), msg)

    def test_setDefault(self):
        ph = self.ph1
        ph.addParam("b", int, params.ParamTypes.Required)
        ph.specify("a", 2)
        ph.specify("b", 2)
        msg = """
        setDefault should set the default value for the specified key
        """
        ph.setDefault("a")
        self.assertEqual({"a": [1], "b": [2]}, paramMapValues(ph), msg)

        msg = """
        setDefault should set the default values if key is a list"""
        ph.specify("a", 2)
        ph.setDefault(["a"])
        self.assertEqual({"a": [1], "b": [2]}, paramMapValues(ph), msg)
        
        msg = """
        setDefault should set the default values for all keys if key
        is not specified"""
        ph.specify("a", 2)
        ph.setDefault()
        self.assertEqual({"a": [1], "b": []}, paramMapValues(ph), msg)


    def test_getParamValue(self):
        ph1 = self.ph1
        ph1.addParam("a", int, params.ParamTypes.Required)
        msg = """
        getParamValue should return None if there is no parameter for 
        the key"""
        self.assertEqual(None, ph1.getParamValue("b"), msg)
         
        msg = """
        getParamValue should return the first value of the parameter"""
        ph1.specify("a", [1, 2])
        self.assertEqual(1, ph1.getParamValue("a"), msg)

    def test_getParamValues(self):
        ph1 = self.ph1
        ph1.addParam("a", int, params.ParamTypes.Required)
        ph1.addParam("b", [1.1, 2.0], params.ParamTypes.Required)
        values = ph1.getParamValues("a")
        msg = """
        getParamValues should return an empty list if parameter is
        not specified"""
        self.assertEqual([], values, msg)

        msg = """
        getParamValues should return the parameter values as a list"""
        values = ph1.getParamValues("b")
        self.assertEqual([1.1, 2.0], values, msg)


    def test_getElementParams(self):
        ph1 = self.ph1
        e1 = we.Element("Type1")
        e2 = we.Element("Type2")
        ph1.addParam("e1", e1, params.ParamTypes.Optional)
        ph1.addParam("e2", e2, params.ParamTypes.Optional)
        elements = {k: v.value for k, v in ph1.getElementParams().items()}
        self.assertEqual({"e1": e1, "e2": e2}, elements)
        
    def test_getParamMapFiltered(self):
        ph1 = self.ph1
        ph1.addParam("b", "hello", params.ParamTypes.Optional)
        ph1.addParam("c", 1.1, params.ParamTypes.Inferred)
        param_dict = ph1.getParamMapFiltered(params.ParamTypes.Required)
        value_dict = self._toValues(param_dict)
        msg = """
        getParamMapFiltered should return a dict with the parameters
        matching the type_filter"""
        self.assertEqual({"a": [1]}, value_dict, msg)
        
        msg = """
        getParamMapFiltered should return a dict with the parameters
        matching the type_filter when the filter is a list"""
        param_dict = ph1.getParamMapFiltered(
                [params.ParamTypes.Inferred, params.ParamTypes.Optional])
        value_dict = self._toValues(param_dict) 
        self.assertEqual({"b": ["hello"], "c": [1.1]}, value_dict, msg)

class TestParam(unittest.TestCase):
    def test_hasChanges(self):
        p = params.Param("MyProp", "", 0, params.ParamTypes.Required)
        self.assertEqual(0, p.value)
        msg = """
        hasChanges should return False if Parameter has not changed
        recently"""
        t = p.last_update
        self.assertEqual(False, p.hasChanges(t), msg)

        msg = """
        hasChanges should return True if Parameter has changed 
        recently"""
        p.value = 1
        self.assertEqual(True, p.hasChanges(t), msg)
    
    def test_toElement(self):
        p = params.Param("MyProp", "", 0, params.ParamTypes.Required)
        element = p.toElement()
        self.assertEqual(
                int, element.getProperty("skiros:DataType").dataType())
        self.assertEqual([0], element.getProperty("skiros:Default").values)
        self.assertEqual([0], element.getProperty("skiros:Value").values)

        e = we.Element("MyType", eid = "1")
        p = params.Param("MyElement", "", e, params.ParamTypes.Required)
        element = p.toElement()
        self.assertEqual(
                ["MyType"], element.getProperty("skiros:DataType").values)
        
        expected = {"src": "-1", "dst": "1", "type": "skiros:hasValue",
                "state": True, "abstract": False}
        self.assertEqual(
                expected, element.getRelation("-1", "skiros:hasValue", "1")) 

if __name__ == "__main__":
    unittest.main()
