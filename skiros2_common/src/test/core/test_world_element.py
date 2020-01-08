import unittest
from skiros2_common.core.world_element import Element

class TestWorldElement(unittest.TestCase):
    def setUp(self):
        self.e = Element()

    def test_element(self):
        e = Element()
        e.setProperty("Hello", float)
        e.setProperty("Hello", 0.0)
        self.assertEqual(e.getProperty("Hello").value, 0.0)
        
        e.getProperty("Hello").values = 1.0 
        self.assertEqual(e.getProperty("Hello").values , [1.0])

        e.getProperty("Hello").value = 2.0
        self.assertEqual(e.getProperty("Hello").values, [2.0])

    def test_getIdNumber(self):
        msg = """
        getIdNumber should return -1 if element has no specified id"""
        self.assertEqual(-1, self.e.getIdNumber(), msg)
        
        msg = """
        getIdNumber should return the elements specified id"""
        self.e.setUri(4)
        self.assertEqual(4, self.e.getIdNumber(), msg)
   
    def test_getRelations(self):
        relations_list = [["1", "pred1", "2"], ["-1", "pred2", "3"]]
        self.e.addRelation(*relations_list[0])
        self.e.addRelation(*relations_list[1])

        msg = """
        getRelations with no argument should return all relations"""
        expected = self._relationsDefault(relations_list)
        self.assertEqual(expected, self.e.getRelations(), msg)

        msg = """
        getRelations should return a list of relations matching subj 
        pred and obj"""
        expected = self._relationsDefault(relations_list[:1])
        result = self.e.getRelations(*relations_list[0]) 
        self.assertEqual(expected, result, msg)
         
    def test_setRelation(self):
        relations_list = [["-1", "pred", "2"], ["1", "pred", "2"], 
                ["-1", "pred", "3"]]
        self.e.addRelation(*relations_list[0])
        self.e.addRelation(*relations_list[1])
        self.e.setRelation(*relations_list[2])
        msg = """
        When subj or obj are -1 setRelation should only overwrite
        previous relations which also have subj or obj as -1 and
        the same predicate"""
        expected = self._relationsDefault(relations_list[1:])
        self.assertEqual(expected, self.e.getRelations(), msg)

        msg = """
        setRelation should overwrite previous relations with same 
        predicate when subj or obj are not -1""" 
        self.e.setRelation("1", "pred", "2")
        expected = [self._relationDefault("1", "pred", "2")]
        self.assertEqual(expected, self.e.getRelations(), msg)

    def test_addRelation(self):
        msg = """
        addRelation should have no effect if relation is already 
        added"""
        relations_list = [["1", "pred", "2"], [u"2", "pred", u"3"]]
        self.e.addRelation(*relations_list[0])
        self.e.addRelation(*relations_list[0])
        expected = self._relationsDefault(relations_list[:1])
        self.assertEqual(expected, self.e.getRelations(), msg)
        
        # test pyhton 2 compatibility
        msg = """
        addRelation should add the relation if subj and obj are unicode
        strings"""
        self.e.addRelation(*relations_list[1])
        expected = self._relationsDefault(relations_list)
        self.assertEqual(expected, self.e.getRelations())

    def test_hasProperty(self):
        e = Element()
        e.setProperty("Integer", "2", "xsd:int")
        self.assertEqual(False, e.hasProperty("NonExistingKey"))
        self.assertEqual(False, e.hasProperty("Integer", 1))
        self.assertEqual(True, e.hasProperty("Integer", 2))
        self.assertEqual(True, e.hasProperty("Integer"))
        e.setProperty("Type", int)
        self.assertEqual(False, e.hasProperty("Type", not_none = True))
        
    def test_setProperty(self):
        e = Element()
        e.setProperty("Integer", 1)
        self.assertEqual([1], e.getProperty("Integer").values)

        msg = """
        setProperty should convert when setting the property if 
        force_convertion is true"""        
        e.setProperty("Integer", "2", force_convertion = True)
        self.assertEqual([2], e.getProperty("Integer").values, msg)
        
        msg = """
        setProperty should convert when setting the property if 
        value is a list and if force_convertion is true"""
        e.setProperty("Integer", ["1", "2"], force_convertion = True)
        self.assertEqual([1, 2], e.getProperty("Integer").values, msg)

        msg = """
        setProperty should convert unicode values to str before
        creating new property"""
        e.setProperty("strKey", u"a")
        self.assertEqual(str, type(e.getProperty("strKey").value), msg)
        
        msg = """
        setProperty should convert unicode values to str when setting
        the property"""
        e.setProperty("strKey", u"b")
        self.assertEqual(["b"], e.getProperty("strKey").values, msg)
        
    def test_appendProperty(self):
        e = Element()
        e.setProperty("Integer", 2, "xsd:int")
        self.assertEqual(2, e.getProperty("Integer").value)
        e.appendProperty("Integer", 3)
        self.assertEqual([2, 3], e.getProperty("Integer").values)

    def _hasRelations(self, element, relation_arg_list):
        for r in relation_arg_list:
            if not element.hasRelation(*r):
                return False
        return True

    # return a relation with state and abstract set to their default values
    def _relationDefault(self, subj, pred, obj):
        return {'src': subj, 'dst': obj, 'type': pred, 'state': True,
                'abstract': False} 

    # return a list of relations with state and abstract set to their 
    # default values  
    def _relationsDefault(self, relation_arg_list):
        return [self._relationDefault(*r) for r in relation_arg_list]

if __name__ == '__main__':
    unittest.main()
