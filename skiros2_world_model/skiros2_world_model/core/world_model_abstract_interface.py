class WmException(Exception):
    pass

class OntologyAbstractInterface:
    def lock(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def unlock(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def query_ontology(self, query, cut_prefix=False):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def set_default_prefix(self, default_prefix):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_individual(self, name):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def add_individual(self, element, ontology_name):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def add_prefix(self, parent_class):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def remove_prefix(self, parent_class):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_type(self, uri):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_sub_classes(self, parent_class, recursive=False):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_sub_properties(self, parent_property, recursive=False):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")


class WorldModelAbstractInterface:
    def add_element(self, e):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def update_element(self, e):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def remove_element(self, eid):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def set_relation(self, subj_id, predicate, obj_id, value=True):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def resolve_element(self, e):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_template_element(self, individual):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_element(self, eid):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def getChildElements(self, e, relation_filter="", type_filter=""):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def get_branch(self, root_id, relation_filter="", type_filter=""):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def queryModel(self, subject_id=-1, predicate="", object_id=-1):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")
