import unittest
from skiros2_common.core.world_element import Element
import skiros2_common.core.params as params
from skiros2_common.core.conditions import ConditionProperty

class TestConditionProperty(unittest.TestCase):

    def test_ConditionProperty(self):
        ph = params.ParamHandler()
        e = Element('Type')
        e.setProperty('Float', 0.0)
        ph.addParam('Param1', e, params.ParamTypes.Required)
        equalZero = ConditionProperty('Example', 'Float', 'Param1', '=', 0.0, True)
        self.assertEqual(True, equalZero.evaluate(ph, None))
        majorZero = ConditionProperty('Example', 'Float', 'Param1', '>', 0.0, True)
        self.assertEqual(False, majorZero.evaluate(ph, None))
        majorMinusOne = ConditionProperty('Example', 'Float', 'Param1', '>', -1.0, True)
        self.assertEqual(True, majorMinusOne.evaluate(ph, None))

if __name__ == "__main__":
    unittest.main()

