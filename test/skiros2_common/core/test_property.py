from skiros2_common.core.property import Property
import unittest

class TestProperty(unittest.TestCase):
   
    def setUp(self):
        self.type_property   = Property("key", float)
        self.list_property   = Property("key", [1, 2, 3, 1, -1])
        self.assertEqual([1, 2, 3, 1, -1], self.list_property.values)
        
    def test_isList(self):
        p = Property("key", 1)
        self.assertEqual(False, p.isList())
        p.addValue(2)
        self.assertEqual(True, p.isList())
    
    def test_find(self):
        p = self.list_property
        self.assertEqual(0, p.find(1))
        self.assertEqual(4, p.find(-1))
        self.assertEqual(-1, p.find(5))
        self.assertEqual(-1, p.find(1.1))

    def test_removeValue(self): 
        p = self.list_property
        p.removeValue(1)
        self.assertEqual([2, 3, 1, -1], p.values)
        p.removeValue(-1)
        self.assertEqual([2, 3, 1], p.values)
        p.removeValue(10)
        self.assertEqual([2, 3, 1], p.values)

    def test_setValue(self):
        p = self.list_property
        p.setValue(13, -1)
        self.assertEqual([1, 2, 3, 1, 13], p.values)
        
        p.setValue("hello", 1)
        self.assertEqual([1, 2, 3, 1, 13], p.values)
        
        p.unset()
        p.setValue(1, 0)
        self.assertEqual([1], p.values)

    def test_setValues(self):
        p = self.type_property
        p.setValues([1.1, "hello"])
        self.assertEqual([], p.values)
        
        p.setValues(1.1)
        self.assertEqual([1.1], p.values)
        
        p.setValues([2.2, 1.1])
        self.assertEqual([2.2, 1.1], p.values)
        
        p.setValues([])
        self.assertEqual([], p.values)

    def test_unset(self):
        p = self.list_property
        p.unset()
        self.assertEqual(None, p.getValue())
        self.assertEqual(False, p.isSpecified())

    def test_isSpecified(self):
        self.assertEqual(False, self.type_property.isSpecified()) 
        self.assertEqual(True, self.list_property.isSpecified())

    def test_addValue(self):
        tp = self.type_property
        tp.addValue("hello")
        self.assertEqual(None, tp.value)
        
        tp.addValue(1.1)
        self.assertEqual(1.1, tp.value)
        
        tp.addValue(2.2)
        self.assertEqual([1.1, 2.2], tp.values)

    def test_setValueFromStr(self):
        dict_prop = Property("key", dict)
        dict_prop.setValueFromStr("{'hello': 1}")
        self.assertEqual({'hello': 1}, dict_prop.value)
        
if __name__ == '__main__':
    unittest.main()
