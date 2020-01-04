import unittest
from skiros2_common.core.world_element import Element
import skiros2_common.core.params as params
from skiros2_common.core.conditions import (
        ConditionProperty, ConditionHasProperty, ConditionOr,
        ConditionIsSpecified)

class ConditionTestCase(unittest.TestCase):
    def setUp(self):
        e = Element('Type')
        e.setProperty('Float', 0.0)
        self.e = e
        ph = params.ParamHandler()
        ph.addParam('Param1', e, params.ParamTypes.Required)
        self.ph = ph
        self.equalZero = ConditionProperty(
                'Example', 'Float', 'Param1', '=', 0.0, True)
        self.majorZero = ConditionProperty(
                'Example', 'Float', 'Param1', '>', 0.0, True)

class TestConditionOr(ConditionTestCase):
    def test_ConditionOr(self):
        e, ph = self.e, self.ph
        greaterOrEqualZero = ConditionOr(True)
        greaterOrEqualZero.addCondition(self.majorZero)
        greaterOrEqualZero.addCondition(self.equalZero)
        self.assertEqual(True, greaterOrEqualZero.evaluate(ph, None))
        e.setProperty('Float', 1.0)
        self.assertEqual(True, greaterOrEqualZero.evaluate(ph, None))
        e.setProperty('Float', -1.0)
        self.assertEqual(False, greaterOrEqualZero.evaluate(ph, None))


class TestConditionProperty(ConditionTestCase):
    def test_ConditionProperty(self):
        e, ph = self.e, self.ph
        self.assertEqual(True, self.equalZero.evaluate(ph, None))
        self.assertEqual(False, self.majorZero.evaluate(ph, None))
        
        majorMinusOne = ConditionProperty(
                'Example', 'Float', 'Param1', '>', -1.0, True)
        self.assertEqual(True, majorMinusOne.evaluate(ph, None))


class TestConditionHasProperty(ConditionTestCase):
    def test_conditionHasProperty(self):
        e, ph1, = self.e, self.ph
        ph2 = params.ParamHandler()
        ph2.addParam('Param1', e, params.ParamTypes.Optional)
        cond = ConditionHasProperty('Example', 'Float', 'Param1', True)
        self.assertEqual(False, cond.evaluate(ph1, None))
        self.assertEqual(True, cond.evaluate(ph2, None))

        e.setUri("1")
        self.assertEqual(True, cond.evaluate(ph1, None))
        
        e.getProperty('Float').unset()
        self.assertEqual(False, cond.evaluate(ph1, None))
        
        e.removeProperty('Float')
        self.assertEqual(False, cond.evaluate(ph1, None))


class TestConditionIsSpecified(ConditionTestCase):
    def test_conditionIsSpecified(self):
        e, ph = self.e, self.ph
        cond = ConditionIsSpecified('Example', 'Param1', True)
        self.assertEqual(False, cond.evaluate(ph, None))
        e.setUri('1')
        self.assertEqual(True, cond.evaluate(ph, None))


if __name__ == "__main__":
    unittest.main()

