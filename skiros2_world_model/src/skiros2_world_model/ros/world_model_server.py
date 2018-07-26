import rospy
import rospkg
from os import walk
import skiros2_common.tools.logger as log
import skiros2_common.ros.utils as utils
import skiros2_msgs.msg as msgs
import skiros2_msgs.srv as srvs
from skiros2_common.tools.plugin_loader import PluginLoader
from skiros2_common.core.discrete_reasoner import DiscreteReasoner
from skiros2_world_model.ros.ontology_server import OntologyServer
from skiros2_world_model.core.world_model import WorldModel, Element
import uuid
from time import sleep

class WorldModelServer(OntologyServer):
    def __init__(self, anonymous=False):
        self._monitor = None
        rospy.init_node("wm", anonymous=anonymous)
        rospy.on_shutdown(self._wait_clients_disconnection)
        self._verbose = rospy.get_param('~verbose', False)
        self._wm = WorldModel(self._verbose, self.wm_change_cb)
        self._ontology = self._wm
        self._plug_loader = PluginLoader()
        self._init_wm()
        self._load_reasoners()
        #================Snapshot======================
        self._curr_snapshot = uuid.uuid4() # random UUID
        #self._snapshots_log = []
        #================ROS======================
        self._set_relation = rospy.Service('~scene/set_relation', srvs.WmSetRelation, self._wm_set_rel_cb)
        self._query_relations = rospy.Service('~scene/query_relations', srvs.WmQueryRelations, self._wm_query_rel_cb)
        self._get = rospy.Service('~scene/get', srvs.WmGet, self._wm_get_cb)
        self._modify = rospy.Service('~scene/modify', srvs.WmModify, self._wm_modify_cb)
        self._monitor = rospy.Publisher("~monitor", msgs.WmMonitor, queue_size=20, latch=True)
        self._load_and_save = rospy.Service('~ontology/load_and_save', srvs.WoLoadAndSave, self._load_and_save_cb)
        self.init_ontology_services()

    def _init_wm(self):
        rospack = rospkg.RosPack()
        self._skiros_dir = rospack.get_path('skiros2')+'/owl'
        self._workspace = rospy.get_param('~workspace_dir', self._skiros_dir)
        for (dirpath, dirnames, filenames) in walk(self._skiros_dir):
            for name in filenames:
                if name.find('.owl')>=0:
                    self._wm.load(dirpath+'/'+name)
        for (dirpath, dirnames, filenames) in walk(self._workspace):
            for name in filenames:
                if name.find('.owl')>=0:
                    self._wm.load(dirpath+'/'+name)
        if not self._workspace:
            self._workspace = self._skiros_dir
        self._wm.set_workspace(self._workspace)
        log.info("[{}]".format(self.__class__.__name__), "Workspace folder: {}".format(self._workspace))
        self._wm.set_default_prefix('skiros', 'http://rvmi.aau.dk/ontologies/skiros.owl#')
        for prefix, uri1 in self._wm._ontology.namespace_manager.store.namespaces():
            if prefix.find("default")>-1:
                self._wm._bind(prefix, "")
        self._wm._bind("", "")
        init_scene = rospy.get_param('~init_scene', "")
        if init_scene!="":
            self._wm.load_scene(init_scene)
        else:
            self._wm.reset()

    def _load_reasoners(self):
        """
        Load reasoner plugins
        """
        #Load plugins descriptions
        for package in rospy.get_param('~reasoners_pkgs', []):
            self._plug_loader.load(package, DiscreteReasoner)
        for p in self._plug_loader:
            self._wm.load_reasoner(p)

    def _wait_clients_disconnection(self):
        if self._monitor:
            while self._monitor.get_num_connections()>0:
                sleep(0.1)

    def wm_change_cb(self, author, action, element=None, relation=None):
        if element is not None:
            self._publish_change(author, action, [utils.element2msg(element)])
        else:
            self._publish_change(author, action, relation=utils.relation2msg(relation))

    def _publish_change(self, author, action, elements=None, relation=None):
        msg =  msgs.WmMonitor()
        msg.prev_snapshot_id = self._curr_snapshot.hex
        self._curr_snapshot = uuid.uuid4() # random UUID
        msg.snapshot_id = self._curr_snapshot.hex
        msg.stamp = rospy.Time.now()
        msg.author = author
        msg.action = action
        if elements:
            msg.elements = elements
        if relation:
            msg.relation.append(relation)
        self._monitor.publish(msg)

    def _load_and_save_cb(self, msg):
        if msg.context!='scene':
            return OntologyServer._load_and_save_cb(msg)
        else:
            with self._times:
                if msg.action==msg.SAVE:
                    self._wm.save_scene(msg.filename)
                elif msg.action==msg.LOAD:
                    self._wm.load_scene(msg.filename)
                    self._publish_change("", "reset", elements=[])
            if self._verbose:
                log.info("[wmLoadAndSave]", "{} file {}. Time: {:0.3f} secs".format(msg.action, msg.filename, self._times.getLast()))
            return srvs.WoLoadAndSaveResponse(True)

    def _wm_query_rel_cb(self, msg):
        to_ret = srvs.WmQueryRelationsResponse()
        with self._times:
            to_ret.matches = [utils.relation2msg(x) for x in self._wm.getRelations(utils.msg2relation(msg.relation))]
        if self._verbose:
            log.info("[wmQueryRelation]", "Query: {} Answer: {}. Time: {:0.3f} secs".format(msg.relation, to_ret.matches, self._times.getLast()))
        return to_ret

    def _wm_get_cb(self, msg):
        with self._times:
            to_ret = srvs.WmGetResponse()
            if msg.action == msg.GET:
                to_ret.elements.append(utils.element2msg(self._wm.get_element(msg.element.id)))
            elif msg.action == msg.GET_TEMPLATE:
                to_ret.elements.append(utils.element2msg(self._wm.get_template_individual(msg.element.label)))
            elif msg.action == msg.GET_RECURSIVE:
                for _, e in self._wm.get_recursive(msg.element.id, msg.relation_filter, msg.type_filter).iteritems():
                    to_ret.elements.append(utils.element2msg(e))
            elif msg.action == msg.RESOLVE:
                for e in self._wm.resolve_elements(utils.msg2element(msg.element)):
                    to_ret.elements.append(utils.element2msg(e))
        output = ""
        einput = utils.msg2element(msg.element)
        for e in to_ret.elements:
            output += "{} ".format(e.id)
        if self._verbose:
            log.info("[WmGet]", "Done {} [{}]. Answer: {}. Time: {:0.3f} secs".format(msg.action, einput, output, self._times.getLast()))
        to_ret.snapshot_id = self._curr_snapshot.hex
        return to_ret

    def _wm_set_rel_cb(self, msg):
        with self._times:
            if msg.value:
                temp = "+"
                self._wm.add_relation(utils.msg2relation(msg.relation), msg.author, is_relation=True)
                self._publish_change(msg.author, "add", relation=msg.relation)
            else:
                temp = "-"
                self._wm.remove_relation(utils.msg2relation(msg.relation), msg.author, is_relation=True)
                self._publish_change(msg.author, "remove", relation=msg.relation)
        if self._verbose:
            log.info("[wmSetRelCb]", "[{}] {} Time: {:0.3f} secs".format(temp, msg.relation, self._times.getLast()))
        return srvs.WmSetRelationResponse(True)

    def _wm_modify_cb(self, msg):
        to_ret = srvs.WmModifyResponse()
        with self._times:
            if msg.action == msg.ADD:
                e_list = list()
                for e in msg.elements:
                    updated_e = self._wm.add_element(utils.msg2element(e), msg.author)
                    to_ret.ids.append(updated_e.id)
                    e_list.append(utils.element2msg(updated_e))
                self._publish_change(msg.author, "add", elements=e_list)
            elif msg.action == msg.UPDATE:
                e_list = list()
                for e in msg.elements:
                    self._wm.update_element(utils.msg2element(e), msg.author)
                    to_ret.ids.append(e.id)
                    e_list.append(utils.element2msg(self._wm.get_element(e.id)))
                self._publish_change(msg.author, "update", elements=e_list)
            elif msg.action == msg.UPDATE_PROPERTIES:
                e_list = list()
                for e in msg.elements:
                    self._wm.update_properties(utils.msg2element(e), msg.author, self._wm.get_reasoner(msg.type_filter), False)
                    to_ret.ids.append(e.id)
                    e_list.append(utils.element2msg(self._wm.get_element(e.id)))
                self._publish_change(msg.author, "update", elements=e_list)
            elif msg.action == msg.REMOVE:
                for e in msg.elements:
                    to_ret.ids.append(self._wm.remove_element(utils.msg2element(e), msg.author))
                self._publish_change(msg.author, "remove", elements=msg.elements)
            elif msg.action == msg.REMOVE_RECURSIVE:
                for e in msg.elements:
                    to_ret.ids += self._wm.remove_recursive(utils.msg2element(e), msg.author, msg.relation_filter, msg.type_filter)
                self._publish_change(msg.author, "remove_recursive", elements=[utils.element2msg(Element(eid=cid)) for cid in to_ret.ids])
        if self._verbose:
            log.info("[WmModify]", "{} {} {}. Time: {:0.3f} secs".format(msg.author, msg.action, to_ret.ids, self._times.getLast()))
        return to_ret

    def run(self):
        rospy.spin()
