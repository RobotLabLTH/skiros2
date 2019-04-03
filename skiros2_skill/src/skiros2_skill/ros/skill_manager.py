import rospy
import skiros2_msgs.msg as msgs
import skiros2_msgs.srv as srvs
import skiros2_common.ros.utils as utils
from skiros2_skill.ros.utils import *
import skiros2_world_model.ros.world_model_interface as wmi
import skiros2_skill.core.skill as skill
from skiros2_common.core.abstract_skill import State
from skiros2_skill.core.skill_instanciator import SkillInstanciator
from discovery_interface import DiscoverableNode
import skiros2_common.tools.logger as log
from skiros2_common.core.world_element import Element
from skiros2_common.tools.id_generator import IdGen
from skiros2_common.tools.plugin_loader import *
from multiprocessing.dummy import Process
import skiros2_skill.core.visitors as visitors
from skiros2_common.tools.time_keeper import TimeKeeper
from std_msgs.msg import Empty

log.setLevel(log.INFO)

def skill2msg(skill):
    msg = msgs.ResourceDescription()
    msg.type = skill._type
    msg.name = skill._label
    msg.params = utils.serializeParamMap(skill._description._params.getParamMap())
    return msg

class BtTicker:
    """
    Manager of a set of Behavior Trees (Tasks) and a visitor

    Ticks the tasks sequentially, with the specified visitor

    Provides interfaces to start, pause, stop the ticking process and to add/remove tasks
    """
    _verbose = True
    _tasks_to_preempt = list()
    _tasks = {}
    _process = None
    _visitor = None
    _id_gen = IdGen()

    _progress_cb = None
    _tick_cb = None

    _finished_skill_ids = dict()

    def _run(self, _):
        """
        @brief Tick tasks at 25hz
        """
        BtTicker._finished_skill_ids = dict()
        rate = rospy.Rate(25)
        log.info("[BtTicker]", "Execution starts.")
        for uid in list(BtTicker._tasks.keys()):
            t = BtTicker._tasks[uid]
            printer = visitors.VisitorPrint(BtTicker._visitor._wm, BtTicker._visitor._instanciator)
            printer.traverse(t)
            self.publish_progress(uid, printer)
        while BtTicker._tasks:
            self._tick()
            rate.sleep()
            self._tick_cb()
        log.info("[BtTicker]", "Execution stops.")

    def _tick(self):
        visitor = BtTicker._visitor
        for uid in list(BtTicker._tasks.keys()):
            if uid in BtTicker._tasks_to_preempt:
                BtTicker._tasks_to_preempt.remove(uid)
                visitor.preempt()
            t = BtTicker._tasks[uid]
            result = visitor.traverse(t)
            self.publish_progress(uid, visitor)
            if result != State.Running and result != State.Idle:
                self.remove_task(uid)

    def kill(self):
        if not BtTicker._process is None:
            del BtTicker._process
            BtTicker._process = None
            self._tick()

    def is_running(self):
        if BtTicker._process is None:
            return False
        return BtTicker._process.is_alive()

    def publish_progress(self, uid, visitor):
        finished_skill_ids = BtTicker._finished_skill_ids
        for (id,desc) in visitor.snapshot():
            if finished_skill_ids.has_key(id):
                if finished_skill_ids[id]['state'] == desc['state'] and finished_skill_ids[id]['msg'] == desc['msg']:
                    continue
            finished_skill_ids[id] = desc
            self._progress_cb(task_id=uid, id=id, **desc)

    def observe_progress(self, func):
        self._progress_cb = func

    def observe_tick(self, func):
        self._tick_cb = func

    def clear(self):
        if BtTicker._visitor:
            BtTicker._visitor.preempt()
            BtTicker._process.join()
            BtTicker._visitor = None
        BtTicker._tasks.clear()
        BtTicker._id_gen.clear()

    def add_task(self, obj, desired_id=-1):
        uid = BtTicker._id_gen.getId(desired_id)
        obj._label = "task_{}".format(uid)
        BtTicker._tasks[uid] = obj
        return uid

    def remove_task(self, uid):
        BtTicker._tasks.pop(uid)
        BtTicker._id_gen.removeId(uid)

    def start(self, visitor):
        if not self.is_running():
            BtTicker._visitor = visitor
            BtTicker._process = Process(target=BtTicker._run, args=(self, True))
            BtTicker._process.start()
            return True

    def join(self):
        BtTicker._process.join()

    def preempt(self, uid):
        BtTicker._tasks_to_preempt.append(uid)
        starttime = rospy.Time.now()
        timeout = rospy.Duration(5.0)
        while(self.is_running() and rospy.Time.now()-starttime<timeout):
            rospy.sleep(0.1)
        if self.is_running():
            log.info("preempt", "Task {} is not answering. Killing process.".format(uid))
            self.kill()
        log.info("preempt", "Task {} preempted.".format(uid))


class SkillManager:
    """
    The skill manager manage a sub-system of the robot
    """
    def __init__(self, prefix, agent_name, verbose=True):
        self._agent_name = agent_name
        self._wmi = wmi.WorldModelInterface(agent_name, make_cache=True)
        self._wmi.set_default_prefix(prefix)
        self._local_wm = self._wmi
        self._plug_loader = PluginLoader()
        self._instanciator = SkillInstanciator(self._local_wm)
        self._ticker = BtTicker()
        self._verbose = verbose
        self._ticker._verbose = verbose
        self._registerAgent(agent_name)
        self._skills = []
        #self._wmi.unlock() #Ensures the world model's mutex is unlocked

    def observeTaskProgress(self, func):
        self._ticker.observe_progress(func)

    def observeTick(self, func):
        self._ticker.observe_tick(func)

    def _registerAgent(self, agent_name):
        res = self._wmi.resolve_element(Element("cora:Robot", agent_name))
        if res:
            log.info("[{}]".format(self.__class__.__name__), "Found robot {}, skipping registration.".format(res))
            self._robot = res
            for r in self._robot.getRelations("-1", "skiros:hasSkill"):
                self._wmi.remove_element(self._wmi.get_element(r['dst']))
            self._robot = self._wmi.get_element(self._robot.id)
        else:
            self._robot = self._wmi.instanciate(agent_name, True)
            startLocUri = self._wmi.get_template_element(agent_name).getRelations(pred="skiros:hasStartLocation")
            if startLocUri:
                start_location = self._wmi.instanciate(startLocUri[0]["dst"], False, [])
                self._wmi.set_relation(self._robot._id, "skiros:at", start_location._id)
                self._robot = self._wmi.get_element(self._robot.id)
        log.info("[{}]".format(self.__class__.__name__), "Registered robot {}".format(self._robot))
        self._robot.setProperty("skiros:SkillMgr", self._agent_name[self._agent_name.rfind(":")+1:])
        self._wmi.update_element(self._robot)


    def shutdown(self):
        for s in self._skills:
            self._wmi.remove_element(s)
        self._wmi.unlock() #Ensures the world model's mutex gets unlocked

    def loadSkills(self, package):
        """
        Load definitions from a package
        """
        self._plug_loader.load(package, skill.SkillDescription)

    def addSkill(self, name):
        """
        @brief Add a skill to the available skill set
        """
        skill = self._plug_loader.getPluginByName(name)()
        self._instanciator.addInstance(skill)
        e = skill.toElement()
        e.addRelation(self._robot._id, "skiros:hasSkill", "-1")
        #print skill.printInfo(True)
        if not self._wmi.get_type(e.type):
            self._wmi.add_class(e.type, "skiros:Skill")
        self._wmi.add_element(e)
        self._skills.append(e)
        return SkillHolder(self._agent_name, skill.type, skill.label, skill.params.getCopy())

    def addLocalPrimitive(self, name):
        """
        @brief Add a local primitive
        """
        self.addSkill(name)

    def add_task(self, task):
        root = skill.Root("root", self._local_wm)
        for i in task:
            log.info("[SkillManager]", "Add task {}:{} \n {}".format(i.type,i.name,i.ph.printState()))
            root.addChild(skill.SkillWrapper(i.type, i.name, self._instanciator))
            root.last().specifyParamsDefault(i.ph)
        return self._ticker.add_task(root, root.id)

    def preemptTask(self, uid):
        self._ticker.preempt(uid)

    def printTask(self, uid):
        self.visitor = visitors.VisitorPrint(self._local_wm, self._instanciator)
        self.visitor.setVerbose(self._verbose)
        return self._ticker.start(self.visitor)

    def executeTask(self, uid, sim=False, track_params=list()):#[("MotionChange",)]
        self.visitor = visitors.VisitorExecutor(self._local_wm, self._instanciator)
        self.visitor.setSimulate(sim)
        for t in track_params:
            self.visitor.trackParam(*t)
        self.visitor.setVerbose(self._verbose)
        return self._ticker.start(self.visitor)

    def clear_tasks(self):
        self._ticker.clear()

    def executeOptimal(self):
        #Optimize Procedure
        self.optimizeTask()
        self.printTask()
        #Execute
        return self.executeTask(False)

    def simulateTask(self, uid):
        self.visitor = visitors.VisitorReversibleSimulator(self._local_wm, self._instanciator)
        self.visitor.setVerbose(self._verbose)
        #self.visitor.trackParam("Initial")
        #self.visitor.trackParam("Gripper")
        if self.visitor.traverse(self._tasks[uid]):
            self._task = self.visitor.getExecutionRoot()

    def optimizeTask(self):
        self.visitor = optimizer.VisitorOptimizer(self._local_wm, self._instanciator)
        #self.visitor.setVerbose(True)
        #self.visitor.trackParam("PlacingCell")
        #self.visitor.trackParam("Object")
        #rospy.sleep(1.)
        self.publish("Optimization", 1, "Start.")
        try:
            if self.visitor.traverse(self._task):
                self._task = self.visitor.getExecutionRoot()
                return True
            else:
                self._task = self.visitor.getExecutionRoot()
                return False
        except KeyError, e:
            self._task = self.visitor.getExecutionRoot()
            print "Exe: {}".format(self.visitor._execution_branch)
            self.printTask()
            raise e

class SkillManagerNode(DiscoverableNode):
    """
    At boot:
        -add the robot description on the world model
        -load skills and add descriptions on the world model
        -interfaces with resource mgrs and retrieve resource list
        -add resource descriptions on the world model
    Main roles:
        -receive tasks (sequences of skills) and process them with visitors
        -currently available visitors: print, execute, simulate, optimize
        -publish feedback on topic /monitor (TODO)
    """
    def __init__(self):
        rospy.init_node("skill_mgr", anonymous=False)
        robot_name = rospy.get_name()
        prefix = ""
        full_name = rospy.get_param('~prefix', prefix) + ':' + robot_name[robot_name.rfind("/")+1:]
        self._sm = SkillManager(rospy.get_param('~prefix', prefix), full_name, verbose=rospy.get_param('~verbose', True))
        self._sm.observeTaskProgress(self._onProgressUpdate)
        self._sm.observeTick(self._onTick)
        #Init skills
        self._initialized = False
        self._getskills = rospy.Service('~get_skills', srvs.ResourceGetDescriptions, self._getDescriptionsCb)
        self._initSkills()
        #self._sm._wmi.instanciate("skiros:large_box_test_starter", relations=[])
        rospy.sleep(0.5)
        self._initialized = True
        #self._sm._local_wm.sync()
        #self._sm._local_wm.printModel()
        #Start communications
        self._command = rospy.Service('~command', srvs.SkillCommand, self._commandCb)
        self._monitor = rospy.Publisher("~monitor", msgs.SkillProgress, queue_size=20)
        self._tick_rate = rospy.Publisher("~tick_rate", Empty, queue_size=20)
        rospy.on_shutdown(self.shutdown)
        self.init_discovery("skill_managers", robot_name)

    def _initSkills(self):
        """
        @brief Initialize the robot with a set of skills
        """
        for r in rospy.get_param('~libraries_list', []):
            self._sm.loadSkills(r)
        #Instanciate local primitives
        for r in rospy.get_param('~primitive_list', []):
            self._sm.addLocalPrimitive(r)
        sl = rospy.get_param('~skill_list', [])
        if not sl:
            pass #TODO: load all defined skills
        for r in sl:
            self._sm.addSkill(r)

    def _makeTask(self, msg):
        task = []
        for s in msg:
            task.append(SkillHolder("", s.type, s.name, utils.deserializeParamMap(s.params)))
        return task

    def _commandCb(self, msg):
        if msg.action == msg.START:
            task_id = self._sm.add_task(self._makeTask(msg.skills))
            #self._sm.printTask(task_id)
            self._sm.executeTask(task_id)
        elif msg.action == msg.PREEMPT:
            task_id = self._sm.preemptTask(msg.execution_id)
        elif msg.action == msg.PAUSE:
            log.error("[{}]".format(self.__class__.__name__), "TODO")
            return srvs.SkillCommandResponse(False, -1)
        elif msg.action == msg.KILL:
            log.error("[{}]".format(self.__class__.__name__), "TODO")
            return srvs.SkillCommandResponse(False, -1)
        else:
            log.error("[{}]".format(self.__class__.__name__), "Unrecognized command.")
            return srvs.SkillCommandResponse(False, -1)
        return srvs.SkillCommandResponse(True, task_id)

    def _onTick(self):
        self._tick_rate.publish(Empty())

    def _onProgressUpdate(self, *args, **kwargs):
        log.debug("[{}]".format(self.__class__.__name__), "{}:Task[{task_id}]{type}:{label}[{id}]: Message[{code}]: {msg} ({state})".format(self._sm._agent_name[1:], **kwargs))
        msg = msgs.SkillProgress()
        msg.robot = rospy.get_name()
        msg.task_id = kwargs['task_id']
        msg.id = kwargs['id']
        msg.type = kwargs['type']
        msg.label = kwargs['label']
        msg.state = kwargs['state']
        msg.parent_label = kwargs['parent_label']
        msg.parent_id = kwargs['parent_id']
        msg.progress_code = kwargs['code']
        msg.progress_time = kwargs['time']
        msg.progress_message = kwargs['msg']
        self._monitor.publish(msg)

    def _getDescriptionsCb(self, msg):
        """
        Called when receiving a command on ~/get_descriptions
        """
        while not self._initialized:
            rospy.sleep(0.1)
        to_ret = srvs.ResourceGetDescriptionsResponse()
        for k, r in self._sm._instanciator._available_instances.iteritems():
            for s in r:
                to_ret.list.append(skill2msg(s))
        return to_ret

    def shutdown(self):
        self.shutdown_discovery()
        self._sm.shutdown()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = SkillManagerNode()
    node.run()
