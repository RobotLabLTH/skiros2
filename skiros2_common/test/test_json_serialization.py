import unittest
import skiros2_common.ros.utils as utils
import skiros2_common.core.params as param

s_param_map_strings = [
'[param: "{\\"values\\": [], \\"specType\\": 0, \\"type\\": \\"dict\\", \\"description\\": \\"\\", \\"key\\"\\\n  : \\"MyDict\\"}"]',
'[param: "{\\"values\\": [], \\"specType\\": 0, \\"type\\": \\"list\\", \\"description\\": \\"\\", \\"key\\"\\\n  : \\"MyList\\"}", '
+ 'param: "{\\"values\\": [], \\"specType\\": 0, \\"type\\": \\"dict\\", \\"description\\": \\"\\", \\"key\\"\\\n  : \\"MyDict\\"}"]',
'[param: "{\\"values\\": [\\"String\\"], \\"specType\\": 0, \\"type\\": \\"str\\", \\"description\\": \\"\\\n  \\", \\"key\\": \\"MyString\\"}"]']

class TestSerialization(unittest.TestCase):
    def assertSameItems(self, a, b):
        msg = "{} does not have the same items as {}".format(a, b)
        self.longMessage = False
        self.assertEqual(sorted(a), sorted(b), msg)

    def test_serialize_ParamMap(self):
        ph = param.ParamHandler()
        ph.addParam("MyDict", dict, param.ParamTypes.Required)
        s_param_map  = utils.serializeParamMap(ph._params)
        self.assertSameItems(s_param_map_strings[0], str(s_param_map))

        ph.addParam("MyList", list, param.ParamTypes.Required)
        s_param_map  = utils.serializeParamMap(ph._params)
        self.assertSameItems(s_param_map_strings[1], str(s_param_map))

        params = {}
        params["MyDict"] = param.Param("MyDict", "", dict, param.ParamTypes.Required)
        s_param_map = utils.serializeParamMap(params)
        self.assertSameItems(s_param_map_strings[0], str(s_param_map))

        params = {}
        params["MyString"] = param.Param("MyString", "", "String", param.ParamTypes.Required)
        s_param_map = utils.serializeParamMap(params)
        self.assertSameItems(s_param_map_strings[2], str(s_param_map))

    def test_deserializeParamMap(self):
        ph = param.ParamHandler()
        ph.addParam("MyDict", dict, param.ParamTypes.Required)
        d_ph = param.ParamHandler(
                utils.deserializeParamMap(utils.serializeParamMap(ph._params)))
        self.assertEqual('MyDict:[] ', d_ph.printState())

        ph.addParam("MyList", list, param.ParamTypes.Required)
        d_ph = param.ParamHandler(
                utils.deserializeParamMap(utils.serializeParamMap(ph._params)))
        self.assertEqual('MyDict:[] MyList:[] ', d_ph.printState())

    def test_deserializePropertyMap(self):
        params = {}
        params["MyDict"] = param.Property("MyDict", dict)
        params["MyFloat"] = param.Property("MyFloat", float)
        params = utils.deserializePropertyMap(utils.serializePropertyMap(params))
        properties = [p.printState() for p in params.values()]
        self.assertEqual(['MyDict:[]', 'MyFloat:[]'], properties)

        params["MyFloat"].value = 1.0
        properties = [p.printState() for p in params.values()]
        self.assertEqual(['MyDict:[]', 'MyFloat:[1.0]'], properties)

    def test_serializePropertyMap(self):
        params = {}
        params["MyDict"] = param.Property("MyDict", dict)
        s_property_map = utils.serializePropertyMap(params)
        expected = (
                '[key: "MyDict"\n'
                'data_value: "[]"\n'
                'dataType: "dict"]')

        self.assertEqual(expected, str(s_property_map))

        params = {}
        params["MyString"] = param.Property("MyString", "String")
        s_property_map = utils.serializePropertyMap(params)
        expected = (
                '[key: "MyString"\n'
                'data_value: "[\\"String\\"]"\n'
                'dataType: "str"]')

        self.assertEqual(expected, str(s_property_map))


if __name__ == "__main__":
    unittest.main()
