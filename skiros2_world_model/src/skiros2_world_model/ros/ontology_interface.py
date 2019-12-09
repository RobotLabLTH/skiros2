import rospy
import skiros2_msgs.srv as srvs
import skiros2_msgs.msg as msgs
import skiros2_common.ros.utils as utils
import skiros2_common.tools.logger as log
from std_srvs.srv import SetBool, SetBoolRequest
from skiros2_common.core.world_element import Element
from skiros2_world_model.core.world_model_abstract_interface import OntologyAbstractInterface, WmException


class OntologyInterface(OntologyAbstractInterface):
    def __init__(self, author_name="test"):
        """
        @brief      Interface to ontology services

                    This class uses ROS services to access and edit the
                    ontology knowledge on a server node

        @param      author_name  (string) Id used to track changes on
                                 the server
        """
        self._author_name = author_name
        self._lock = rospy.ServiceProxy('wm/lock', SetBool)
        self._ontology_query = rospy.ServiceProxy('wm/ontology/query', srvs.WoQuery)
        self._ontology_modify = rospy.ServiceProxy('wm/ontology/modify', srvs.WoModify)
        self._load_and_save = rospy.ServiceProxy('wm/load_and_save', srvs.WoLoadAndSave)
        log.info("[{}] ".format(self.__class__.__name__), "Waiting wm communications...")
        self._ontology_modify.wait_for_service()
        log.info("[{}] ".format(self.__class__.__name__), "Wm communications active.")
        self._def_prefix = ":"
        self._sub_classes_cache = {}
        self._sub_properties_cache = {}

    def is_connected(self):
        """
        @brief      Determines if connected.

        @return     True if connected, False otherwise.
        """
        try:
            rospy.wait_for_service(self._ontology_query.resolved_name, 0.0)
            return True
        except rospy.ROSException:
            return False

    def lock(self):
        """
        @brief      Locks the ontology server mutex to signal exclusive
                    access

                    Note: locking is NOT mandatory and all functions
                    will still work without locking the server

        @return     True if lock service call was successfull
        """
        return self._call(self._lock, SetBoolRequest(True)).success

    def unlock(self):
        """
        @brief      Unlocks the ontology server mutex

        @return     True if unlock service call was successfull
        """
        return self._call(self._lock, SetBoolRequest(False)).success

    def add_class(self, class_uri, parent_uri, context=""):
        """
        @brief      Creates a new class definition

        @param      class_uri   (string) The class uri
        @param      parent_uri  (string) URI used as parent class

        @return     True if successfully created, False otherwise
        """
        req = srvs.WoModifyRequest()
        req.context = context
        req.statements.append(utils.makeStatementMsg(class_uri, "rdf:type", "owl:class", True))
        req.statements.append(utils.makeStatementMsg(class_uri, "rdfs:subClassOf", parent_uri, True))
        res = self._call(self._ontology_modify, req)
        if res:
            if res.ok:
                return True
        return False

    def add_datatype(self, data_uri, parent_uri, context=""):
        """
        @brief Creates a new datatype definition
        """
        # TODO:
        raise NotImplementedError("NOT IMPLEMENTED.")

    def add_individual(self, element, context=""):
        """
        @brief Creates a new individual
        """
        # TODO:
        raise NotImplementedError("NOT IMPLEMENTED.")

    def add_ontology(self, uri):
        """
        @brief      Creates a new ontology.

        @param      uri   the id of the ontology, that can be used as a
                          context id in other functions

        @return     True if successfully created, False otherwise
        """
        req = srvs.WoModifyRequest()
        req.context = uri
        req.statements.append(utils.makeStatementMsg(uri, "rdf:type", "owl:Ontology", True))
        res = self._call(self._ontology_modify, req)
        if res:
            if res.ok:
                return True
        return False

    def load(self, filename="", context='scene'):
        """
        @brief      Loads a file in a context

                    If no filename is provided use the default name
                    specified when booting the world model

        @param      filename  (string) The filename
        @param      context   (string) The context id

        @return     True if successfully loaded, False otherwise
        """
        msg = srvs.WoLoadAndSaveRequest()
        msg.action = msg.LOAD
        msg.filename = filename
        msg.context = context
        res = self._call(self._load_and_save, msg)
        if(res):
            return res.ok
        return False

    def save(self, filename="", context='scene'):
        """
        @brief      Saves a context in a file

        @param      filename  (string) The filename
        @param      context   (string) The context id

        @return     True if successfully saved, False otherwise
        """
        msg = srvs.WoLoadAndSaveRequest()
        msg.action = msg.SAVE
        msg.filename = filename
        msg.context = context
        res = self._call(self._load_and_save, msg)
        if(res):
            return res.ok
        return False

    def query_ontology(self, query, cut_prefix=True, context=""):
        """
        @brief Direct SPARQL interface. query should be a string in SPARQL syntax

        @param cut_prefix If True the prefix in returned values is removed
        @param context The context in which executing the query

        @return a list of strings
        """
        req = srvs.WoQueryRequest()
        req.query_string = query
        req.context = context
        req.cut_prefix = cut_prefix
        return self._call(self._ontology_query, req).answer

    def set_default_prefix(self, default_prefix):
        self._def_prefix = default_prefix

    def add_prefix(self, uri):
        """
        @brief      Formats the uri for a SPARQL query

                    Adds prefix to uri or does nothing if uri has
                    already the prefix

                    Adds < > brakets, if necessary

        @param      uri   (string) Unformatted uri

        @return     (string) Formatted uri
        """
        if uri.find("#") is -1 and uri.find(":") is -1:
            uri = self._def_prefix + ":" + uri
        elif not uri.find("#") is -1 and uri.find("<") is -1:
            uri = "<" + uri + ">"
        elif not uri.find(":") is -1 and uri.find(":") is 0:
            uri = self._def_prefix + uri
        return uri

    def remove_prefix(self, uri):
        """
        @brief      Removes a prefix from un uri.

        @param      uri   (string) The uri

        @return     (string) uri without prefix
        """
        if uri.find("#") >= 0:
            return uri.split("#")[-1]
        elif uri.find(":") >= 0:
            return uri.split(":")[-1]
        else:
            return uri

    def get_individuals(self, parent_class, recursive=True):
        """
        @brief      Get all individuals of the type of the parent_class

        @param      parent_class  (string) The class of individuals
        @param      recursive     (bool) If recursive, returns also
                                  individuals of subclasses

        @return     list(string) The individuals.
        """
        if recursive:
            return self.query_ontology("SELECT ?x WHERE { ?x rdf:type/rdfs:subClassOf* " + self.add_prefix(parent_class) + " . } ")
        else:
            return self.query_ontology("SELECT ?x where {?x rdf:type+ " + self.add_prefix(parent_class) + "}")

    def get_type(self, uri):
        """
        @brief Returns the type of an individual/class.
        """
        return self.query_ontology("SELECT ?x where {" + self.add_prefix(uri) + " rdf:type ?x}")

    def get_types(self, uri):
        """
        @brief Returns all types of a class
        """
        return self.query_ontology("SELECT ?x where {{ ?x rdf:type {} }}".format(self.add_prefix(uri)))

    def get_triples(self, subj=None, pred=None, obj=None):
        """
        @brief Returns matching triples.

        Note: at least one between subj, pred or obj must left blank for this function to work.
        """
        if subj:
            subj = self.add_prefix(subj)
        else:
            subj = "?x"
        if pred:
            pred = self.add_prefix(pred)
        else:
            pred = "?y"
        if obj:
            obj = self.add_prefix(obj)
        else:
            obj = "?z"
        return self.query_ontology("SELECT * WHERE { " + "{} {} {}".format(subj, pred, obj) + " . } ")

    def get_super_class(self, child_class):
        """
        @brief Returns the parent class of child_class
        """
        to_ret = self.query_ontology("SELECT ?x WHERE { " + self.add_prefix(child_class) + " rdfs:subClassOf ?x. } ")
        if not to_ret:
            log.error("[get_super_class]", "No super class found for {}".format(child_class))
        return to_ret[0]

    def get_sub_classes(self, parent_class, recursive=True):
        """
        @brief Returns the child classes of parent_class. If recursive=True, returns also sub childs classes
        """
        if(parent_class in self._sub_classes_cache):
            return self._sub_classes_cache[parent_class]
        if recursive:
            to_ret = self.query_ontology("SELECT ?x WHERE { ?x rdfs:subClassOf* " + self.add_prefix(parent_class) + " . } ")
        else:
            to_ret = self.query_ontology("SELECT ?x WHERE { ?x rdfs:subClassOf " + self.add_prefix(parent_class) + " . } ")
        self._sub_classes_cache[parent_class] = to_ret
        return to_ret

    def get_sub_properties(self, parent_property, recursive=True):
        """
        @brief Returns the child properties of parent_property. If recursive=True, returns also sub childs properties
        """
        if(parent_property in self._sub_properties_cache):
            return self._sub_properties_cache[parent_property]
        if recursive:
            to_ret = self.query_ontology("SELECT ?x WHERE { ?x rdfs:subPropertyOf* " + self.add_prefix(parent_property) + " . } ")
        else:
            to_ret = self.query_ontology("SELECT ?x WHERE { ?x rdfs:subPropertyOf " + self.add_prefix(parent_property) + " . } ")
        self._sub_properties_cache[parent_property] = to_ret
        return to_ret

    def get_datatype(self, property):
        """
        @brief Returns the property datatype restriction
        """
        answer = self.query_ontology("SELECT ?z WHERE {?x ?t " + property + ". ?x rdf:type owl:Restriction. ?x owl:onDataRange ?z.}")
        return answer[0] if answer else None

    def _call(self, service, msg):
        try:
            resp1 = service(msg)
            return resp1
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
            raise WmException("Service call failed: %s" % e)
