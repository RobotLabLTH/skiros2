import skiros2_common.tools.logger as log
import skiros2_msgs.srv as srvs
from std_srvs.srv import SetBool
from skiros2_world_model.core.ontology_rdflib import Ontology
from skiros2_common.tools.time_keeper import TimeKeeper
import skiros2_common.ros.utils as utils
from threading import Lock
from pyparsing import ParseException

import rclpy
from rclpy.node import Node


class OntologyServer(Node):
    def __init__(self, name='ontology'):
        super().__init__(name)
        self.declare_parameter("verbose", False)
        self._verbose = self.get_parameter('verbose').value
        self._ontology = Ontology()
        self.init_ontology_services()

    def init_ontology_services(self):
        self._times = TimeKeeper()
        self._mutex = Lock()
        self._mutex_srv = self.create_service(SetBool, '~/lock', self._lock_cb)
        self._query = self.create_service(srvs.WoQuery, '~/ontology/query', self._wo_query_cb)
        self._modify = self.create_service(srvs.WoModify, '~/ontology/modify', self._wo_modify_cb)

    def _lock_cb(self, msg, resp):
        """
        @brief Can be used to get sync access to server
        """
        if msg.data:
            self._mutex.acquire()
        else:
            try:
                self._mutex.release()
            except BaseException:
                return resp(False, "Mutex already unlocked.")
        return resp(True, "Ok")

    def _wo_query_cb(self, msg, to_ret):
        try:
            log.assertInfo(self._verbose, "[WoQuery]", "Query: {}. Context: {}".format(msg.query_string, msg.context))
            with self._times:
                for s in self._ontology.query(msg.query_string, context_id=msg.context):
                    temp = ""
                    for r in s:
                        if r is None:
                            continue
                        elif msg.cut_prefix:
                            temp += self._ontology.uri2lightstring(r)
                        else:
                            temp += r.n3()
                        if len(s) > 1:
                            temp += " "
                    to_ret.answer.append(temp)
            log.assertInfo(self._verbose, "[WoQuery]", "Answer: {}. Time: {:0.3f} sec".format(
                to_ret.answer, self._times.get_last()))
        except (AttributeError, ParseException) as e:
            # TODO: test if the bug is fixed, and remove the exception handling
            log.error("[WoQuery]", "Parse error with following query: {}.".format(msg.query_string))
            raise e
        return to_ret

    def _wo_modify_cb(self, msg, resp):
        with self._times:
            for s in msg.statements:
                if s.value:
                    self._ontology.add_relation(utils.msg2relation(s.relation), msg.context, msg.author)
                else:
                    self._ontology.remove_relation(utils.msg2relation(s.relation), msg.context, msg.author)
        log.assertInfo(self._verbose, "[WoModify]", "Done in {} sec".format(self._times.get_last()))
        resp.ok = True
        return resp

    def run(self):
        rclpy.spin(self)
