#################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Francesco Rovida
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################################

import rospy
import skiros2_msgs.msg as msgs
import skiros2_msgs.srv as srvs
import skiros2_common.ros.utils as utils
from skiros2_skill.ros.utils import *
import skiros2_world_model.core.local_world_model as wm
import skiros2_world_model.ros.world_model_interface as wmi
import skiros2_skill.core.skill as skill
from skiros2_common.core.abstract_skill import State
from skiros2_skill.core.skill_instanciator import SkillInstanciator
from skiros2_skill.ros.ros_skill import RosSkill
import skiros2_common.tools.logger as log
from skiros2_common.tools.id_generator import IdGen
from skiros2_common.tools.plugin_loader import *
from multiprocessing.dummy import Process
import skiros2_skill.core.visitors as visitors
from skiros2_resource.ros.resource_layer_interface import *

log.setLevel(log.INFO)

def skill2msg(skill):
    msg = msgs.ResourceDescription()
    msg.type = skill._type
    msg.name = skill._label
    msg.params = utils.serializeParamMap(skill._description._params.getParamMap())
    return msg

class TaskManager:
    """
    Internal task manager used to keep track of on-going tasks
    """
    _verbose = True
    _tasks = {}
    _processes = {}
    _visitors = {}
    _id_gen = IdGen()

    _progress_cb = None


    def __getitem__(self, key):
        return TaskManager._tasks[key]

    def _run(self, uid, visitor, clear):
        """
        Tick tree at 25hz
        """
        progress = visitors.VisitorProgress()
        finished_skill_ids = []

        iteration = 0
        result = State.Running
        rate = rospy.Rate(50)
        while result==State.Running:
            iteration += 1
            result = visitor.traverse(TaskManager._tasks[uid])
            log.debug("[{}]".format(visitor.__class__.__name__), "Iteration {} result: {}".format(iteration, result.name))
            progress.reset()
            progress.traverse(TaskManager._tasks[uid])
            for (id,desc) in progress.snapshot():
                if id not in finished_skill_ids:
                    if desc['state'] is State.Success or desc['state'] is State.Failure:
                        finished_skill_ids.append(id)
                    if self._progress_cb is not None:
                        self._progress_cb(task_id=uid, id=id, **desc)
            #log.info("", "Remaining: {}".format(rate.remaining()))#TODO: decrease the loop time.Optimize operations
            rate.sleep()
        #if result==State.Failure:
        print "===Final state==="
        printer = visitors.VisitorPrint(visitor._wm, visitor._instanciator)
        printer.traverse(TaskManager._tasks[uid])
        self._progress_cb(task_id=uid, id=uid, **{"type":"Task", "label": "-", "state": TaskManager._tasks[uid].state, "msg": "Terminated.", "code": 0})
        if clear:
            self.removeTask(uid)


    def observeProgress(self, func):
        self._progress_cb = func


    def clear(self):
        for uid, v in TaskManager._visitors.iteritems():
            self.preempt(uid)
            self.join(uid)
        TaskManager._tasks.clear()
        TaskManager._visitors.clear()
        TaskManager._id_gen.clear()

    def addTask(self, obj, desired_id=-1):
        uid = TaskManager._id_gen.getId(desired_id)
        TaskManager._tasks[uid] = obj
        return uid

    def removeTask(self, uid):
        if TaskManager._processes.has_key(uid):
            TaskManager._visitors.pop(uid)
            TaskManager._processes.pop(uid)
        TaskManager._tasks.pop(uid)
        TaskManager._id_gen.removeId(uid)

    def start(self, uid, visitor, clear=False):
        TaskManager._visitors[uid] = visitor
        TaskManager._processes[uid] = Process(target=TaskManager._run, args=(self, uid, visitor, clear))
        TaskManager._processes[uid].start()
        return True

    def isAlive(self, uid):
        TaskManager._processes[uid].is_alive()

    def join(self, uid):
        TaskManager._processes[uid].join()

    def preempt(self, uid):
        TaskManager._visitors[uid].preempt()
        log.info("preempt", "Task {} preempted.".format(uid))

    def kill(self, uid):
        log.error("", "TODO")


class SkillManager:
    """
    The skill manager manage a sub-system of the robot
    """
    def __init__(self, prefix, agent_name, verbose=True):
        self._agent_name = agent_name
        self._wmi = wmi.WorldModelInterface(agent_name)
        self._wmi.setDefaultPrefix(prefix)
        self._local_wm = self._wmi
        #self._local_wm._verbose = False
        self._plug_loader = PluginLoader()
        self._instanciator = SkillInstanciator(self._local_wm)
        self._tasks = TaskManager()
        self._verbose = verbose
        self._tasks._verbose = verbose
        self._registerAgent(agent_name)
        self._skills = []

    def observeTaskProgress(self, func):
        self._tasks.observeProgress(func)

    def _registerAgent(self, agent_name):
        res = self._wmi.resolveElement(wm.Element("cora:Robot", agent_name))
        if res:
            log.info("[{}]".format(self.__class__.__name__), "Found robot {}, skipping registration.".format(res))
            self._robot = res
            for r in self._robot.getRelations("-1", "skiros:hasSkill"):
                self._wmi.removeElement(self._wmi.getElement(r['dst']))
            self._robot = self._wmi.getElement(self._robot.id)
        else:
            self._robot = self._wmi.instanciate(agent_name, True)
            startLocUri = self._wmi.getTemplateElement(agent_name).getRelations(pred="skiros:hasStartLocation")
            if startLocUri:
                start_location = self._wmi.instanciate(startLocUri[0]["dst"], False, [])
                self._wmi.setRelation(self._robot._id, "skiros:at", start_location._id)
        log.info("[{}]".format(self.__class__.__name__), "Registered robot {}".format(self._robot))
        self._robot.setProperty("skiros:SkillMgr", self._agent_name[self._agent_name.rfind(":")+1:])
        self._wmi.updateElement(self._robot)


    def shutdown(self):
        for s in self._skills:
            self._wmi.removeElement(s)

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
        if not self._wmi.getType(e._type):
            self._wmi.addClass(e._type, "skiros:Skill")
        self._wmi.addElement(e)
        self._skills.append(e)
        return SkillHolder(self._agent_name, skill._type, skill._label, skill.params.getCopy())

    def addLocalPrimitive(self, name):
        """
        @brief Add a local primitive
        """
        self.addSkill(name)

    def addExternalPrimitive(self, rtype, name, ri, mgr_name):
        """
        @brief Instanciate a link to a primitive hosted on a resource manager
        """
        description = self._plug_loader.getPluginByName(rtype.replace(":", ""))()
        skill = RosSkill()
        skill.setDescription(description, name)
        skill.setRosInterface(ri, mgr_name)
        self._instanciator.addInstance(skill)
        e = skill.toElement()
        e.addRelation(self._robot._id, "skiros:hasSkill", "-1")
        if not self._wmi.getType(e._type):
            self._wmi.addClass(e._type, "skiros:Skill")
        self._wmi.addElement(e)
        self._skills.append(e)

    def addTask(self, task, desired_id=-1):
        root = skill.Root("root", self._local_wm)
        for i in task:
            print i.manager+":"+i.type+":"+i.name+i.ph.printState()
            root.addChild(skill.SkillWrapper(i.type, i.name, self._instanciator))
            root.last().specifyParamsDefault(i.ph)
        return self._tasks.addTask(root, desired_id)

    def preemptTask(self, uid):
        self._tasks.preempt(uid)

    def printTask(self, uid):
        self.visitor = visitors.VisitorPrint(self._local_wm, self._instanciator)
        self.visitor.setVerbose(self._verbose)
        return self._tasks.start(uid, self.visitor)

    def executeTask(self, uid, sim=False, track_params=list()):#[("MotionChange",)]
        self.visitor = visitors.VisitorExecutor(self._local_wm, self._instanciator)
        self.visitor.setSimulate(sim)
        for t in track_params:
            self.visitor.trackParam(*t)
        self.visitor.setVerbose(self._verbose)
        return self._tasks.start(uid, self.visitor)

    def clearTasks(self):
        self._tasks.clear()

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



class SkillManagerNode(object):
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
        self._rli = ResourceLayerInterface()
        self._rli.printState()
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
        self._monitor = rospy.Publisher("skill_managers/monitor", msgs.SkillProgress, queue_size=20)
        rospy.on_shutdown(self._sm.shutdown)
        rospy.sleep(0.5)

    def _initSkills(self):
        """
        @brief Initialize the robot with a set of skills
        """
        for r in rospy.get_param('~libraries_list', []):
            self._sm.loadSkills(r)
        #Instanciate primitives hosted on resource managers
        for mgr, r in self._rli.getAvailableResources():
            self._sm.addExternalPrimitive(r.type, r.name, self._rli, mgr)
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
            self._sm.clearTasks()
            task_id = self._sm.addTask(self._makeTask(msg.skills), msg.execution_id)
            #self._sm.printTask(task_id)
            self._sm.executeTask(task_id)
        elif msg.action == msg.PREEMPT:
            task_id = self._sm.preemptTask(msg.execution_id)
        elif msg.action == msg.PAUSE:
            log.error("[{}]".format(self.__class__.__name__), "TODO")
            pass
        elif msg.action == msg.KILL:
            log.error("[{}]".format(self.__class__.__name__), "TODO")
            pass
        else:
            task_id = self._sm.addTask([SkillHolder("", msg.action, "")])
            self._sm.printTask(task_id)
            #self._sm._tasks.join(task_id)
            self._sm.executeTask(task_id)
            return srvs.SkillCommandResponse(False, -1)
        return srvs.SkillCommandResponse(True, task_id)


    def _onProgressUpdate(self, *args, **kwargs):
        log.debug("[{}]".format(self.__class__.__name__), "{}:Task[{task_id}]{type}:{label}[{id}]: Message[{code}]: {msg} ({state})".format(self._sm._agent_name[1:], **kwargs))
        msg = msgs.SkillProgress()
        msg.robot = self._sm._agent_name
        msg.task_id = kwargs['task_id']
        msg.id = kwargs['id']
        msg.type = kwargs['type']
        msg.label = kwargs['label']
        msg.state = kwargs['state']
        msg.progress_code = kwargs['code']
        msg.progress_message = kwargs['msg']
        self._monitor.publish(msg)

    # def _publishMonitor(self, action, code, description, seconds=0.0):
    #     #start = rospy.Time.now()
    #     #self.publish("Optimization", 1, "Success.", (rospy.Time.now()-start).to_sec())
    #     #self._opt_time = (rospy.Time.now()-start).to_sec()
    #     msg = msgs.ResourceMonitor()
    #     msg.author = "skirospy"
    #     msg.action = action
    #     msg.progress_code = code
    #     msg.progress_description = description
    #     msg.progress_seconds = seconds
    #     self._monitor.publish(msg)

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

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = SkillManagerNode()
    node.run()
