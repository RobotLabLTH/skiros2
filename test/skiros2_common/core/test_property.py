import unittest
from skiros2_common.core.property import Property

class TestProperty(unittest.TestCase):

    def setUp(self):
        self.longMessage = True
        self.type_property   = Property("key", float)
        self.list_property   = Property("key", [1, 2, 3, 1, -1])
        self.assertEqual([1, 2, 3, 1, -1], self.list_property.values)
    
    def test_isList(self):
        msg = """
        isList should return False when Property contains only one
        value"""
        p = Property("key", 1)
        self.assertEqual(False, p.isList(), msg)

        msg = """
        isList should return True when Property contains multiple
        values"""
        p.addValue(2)
        self.assertEqual(True, p.isList(), msg)
    
    def test_find(self):
        p = self.list_property
        
        msg = """
        Failed to find first value"""
        self.assertEqual(0, p.find(1), msg)
        
        msg = """
        Failed to find last value"""
        self.assertEqual(4, p.find(-1), msg)
        
        msg = """
        find should return -1 when value is not in property"""
        self.assertEqual(-1, p.find(5), msg)

    def test_removeValue(self): 
        p = self.list_property
        p.removeValue(1) 
        msg = """
        removeValue should remove first occurance of a value"""
        self.assertEqual([2, 3, 1, -1], p.values, msg)
        p.removeValue(-1)
        
        msg = "Failed to remove last value"
        self.assertEqual([2, 3, 1], p.values, msg)
        p.removeValue(10)
        
        msg = """
        removeValue should have no effect if value is not in property"""
        self.assertEqual([2, 3, 1], p.values, msg)

    def test_setValue(self):
        p = self.list_property
        msg = """
        setValue should append value when called with index -1"""
        p.setValue(13, -1)
        self.assertEqual([1, 2, 3, 1, 13], p.values, msg)
        
        msg = """
        setValue should have not effect if value has the wrong type"""
        p.setValue("hello", 1)
        self.assertEqual([1, 2, 3, 1, 13], p.values, msg)
        
        msg = """
        setValue should append the value when property is not 
        specified"""
        p.unset()
        p.setValue(1, 0)
        self.assertEqual([1], p.values, msg)

    def test_setValues(self):
        msg = """
        setValues should append the values when property is not
        specified"""
        p = self.type_property
        p.setValues(1.1)
        self.assertEqual([1.1], p.values, msg)
        
        msg = """
        setValues should replace the old values with the input values"""      
        p.setValues([2.2, 1.1])
        self.assertEqual([2.2, 1.1], p.values, msg)
        
        msg = """
        setValues should set values to empty list when called with
        empty list"""
        p.setValues([])
        self.assertEqual([], p.values, msg)

        msg = """
        setValues should have no effect when called with a list
        containing values of the wrong type"""
        p.setValues([1.1, "hello"])
        self.assertEqual([], p.values, msg)

    def test_unset(self):
        p = self.list_property
        p.unset()
        msg = """
        unset should remove all the values"""
        self.assertEqual(None, p.getValue(), msg)
        self.assertEqual(False, p.isSpecified(), msg)

    def test_isSpecified(self):
        msg = """
        isSpecified should return False if property contains no values"""
        self.assertEqual(False, self.type_property.isSpecified(), msg) 
        
        msg = """
        isSpecified should return True if property contains values"""
        self.assertEqual(True, self.list_property.isSpecified(), msg)

    def test_addValue(self):
        tp = self.type_property
        tp.addValue("hello")
        msg = """
        addValue should have no effect if value is of the wrong type"""
        self.assertEqual(None, tp.value, msg)
        
        msg = """
        addValue should add the value to a property that is not
        specified"""
        tp.addValue(1.1)
        self.assertEqual(1.1, tp.value)
        
        msg = """
        addValue should append the value to the existing values"""
        tp.addValue(2.2)
        self.assertEqual([1.1, 2.2], tp.values, msg)

    def test_setValueFromStr(self):
        dict_prop = Property("key", dict)
        msg = """
        setValueFromStr should set a value when the string is a dict"""
        dict_prop.setValueFromStr("{'hello': 1}")
        self.assertEqual({'hello': 1}, dict_prop.value, msg)
        
if __name__ == '__main__':
    unittest.main()
