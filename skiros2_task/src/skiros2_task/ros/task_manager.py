import rospy

from std_msgs.msg import Empty

import skiros2_msgs.msg as msgs
import actionlib
import skiros2_common.core.params as skirosp
import skiros2_common.core.conditions as cond

from skiros2_common.tools.decorators import PrettyObject
import skiros2_common.tools.logger as log
import skiros2_common.tools.time_keeper as tk

from copy import deepcopy
from skiros2_common.core.world_element import Element
import skiros2_world_model.ros.world_model_interface as wmi
import skiros2_skill.ros.skill_layer_interface as sli
import skiros2_task.core.pddl_interface as pddl
from skiros2_skill.ros.utils import SkillHolder


class TaskManagerNode(PrettyObject):
    def __init__(self):
        """
        This class manage the robot task planning. A list of goals can be set by
        external agent (e.g. users) with the action server 'task_plan' The task
        manager plans a sequence of skills to reach the goals and returns it.

        Initialize task manager as a ros node. Establish access to the global
        and local world model, skill manager and the task planner.
        """
        rospy.init_node("task_manager", anonymous=False)
        self._author_name = rospy.get_name()

        self._goals = []
        self._skills = {}
        self._abstract_objects = []

        self._wmi = wmi.WorldModelInterface(self._author_name)
        self._sli = sli.SkillLayerInterface(self._author_name)
        self._pddl_interface = pddl.PddlInterface()

        self._verbose = rospy.get_param('~verbose', True)
        log.setLevel(log.INFO)
        self._assign_task_action = actionlib.SimpleActionServer(
            '~task_plan', msgs.AssignTaskAction, execute_cb=self._assign_task_cb, auto_start=False)
        self._assign_task_action.start()

        self._is_ready = False

    @property
    def skills(self):
        """Get available skills.

        Return the updated list of skills available in the system

        Returns:
            dict: {Skill name : instance? }
        """
        if self._sli.has_changes:
            self._skills.clear()
            for ak, e in self._sli._agents.iteritems():
                for sk, s in e._skill_list.iteritems():
                    s.manager = ak
                    self._skills[sk] = s
        return self._skills

    def _assign_task_cb(self, msg):
        """Callback for setting new goals.

        Executed whenever we receive a service call to set a new goal.

        Args:
            msg (skiros2_msgs.srv.TmSetGoals): Service message containing the goals
        """
        try:
            log.info("[Goal]", msg.goals)
            self._current_goals = msg.goals
            plan = self._task_plan()
            log.info("[Plan]", plan)
            if plan is None:
                log.warn(self.class_name, "Planning failed for goals: {}".format(self._current_goals))
                self._result = msgs.AssignTaskResult(1, "Planning failed.")
                self._assign_task_action.set_aborted(self._result)
                return
            if not plan:
                self._result = msgs.AssignTaskResult(2, "No skills to execute.")
                self._assign_task_action.set_succeeded(self._result)
                return
            task = self.build_task(plan)
            self._result = msgs.AssignTaskResult(3, task.toJson())
            self._assign_task_action.set_succeeded(self._result)
            return
        except OSError, e:
            self._result = msgs.AssignTaskResult(1, "FD task planner not found. Maybe is not installed?")
            self._assign_task_action.set_aborted(self._result)
        except Exception, e:
            self._result = msgs.AssignTaskResult(1, str(e))
            self._assign_task_action.set_aborted(self._result)

    def _task_plan(self):  # TODO: make this concurrent
        with tk.Timer(self.class_name) as timer:
            self._pddl_interface.clear()
            self.initDomain()
            timer.toc("Init domain")
            self.setGoal(self._current_goals)
            self.initProblem()
            timer.toc("Init problem")
            plan = self.plan()
            timer.toc("Planning sequence")
            return plan

    def build_task(self, plan):
        """
        @brief Convert the plan (a string) into a tree of parameterized skills
        """
        task = SkillHolder("", "Processor", "Sequential")
        skills = plan.splitlines()
        for s in skills:
            s = s[s.find('(') + 1: s.find(')')]
            tokens = s.split(' ')
            skill = deepcopy(self.skills[tokens.pop(0)])
            planned_map = self._pddl_interface.getActionParamMap(skill.name, tokens)
#            print "{}".format(planned_map)
            for k, v in planned_map.iteritems():
                e = self.get_element(v)
                if e.getIdNumber() < 0:
                    e._id = ""  # ID must be clear if element is not an instance
                skill.ph.specify(k, e)
            task.children.append(skill)
        return task

    def initDomain(self):
        skills = self._wmi.resolve_elements(wmi.Element(":Skill"))
        for skill in skills:
            params = {}
            preconds = []
            holdconds = []
            postconds = []
            # Note: Only skills with pre AND post conditions are considered for planning
            for p in skill.getRelations(pred="skiros:hasParam"):
                e = self._wmi.get_element(p['dst'])
                params[e._label] = e.getProperty("skiros:DataType").value
            for p in skill.getRelations(pred="skiros:hasPreCondition"):
                e = self._wmi.get_element(p['dst'])
                if e.type.find("ConditionRelation") != -1 or e.type == "skiros:ConditionProperty" or e.type == "skiros:ConditionHasProperty":
                    preconds.append(pddl.Predicate(e, params, e.type.find("Abs") != -1))
            for p in skill.getRelations(pred="skiros:hasHoldCondition"):
                e = self._wmi.get_element(p['dst'])
                if e.type.find("ConditionRelation") != -1 or e.type == "skiros:ConditionProperty" or e.type == "skiros:ConditionHasProperty":
                    holdconds.append(pddl.Predicate(e, params, e.type.find("Abs") != -1))
            for p in skill.getRelations(pred="skiros:hasPostCondition"):
                e = self._wmi.get_element(p['dst'])
                if e.type.find("ConditionRelation") != -1 or e.type == "skiros:ConditionProperty" or e.type == "skiros:ConditionHasProperty":
                    postconds.append(pddl.Predicate(e, params, e.type.find("Abs") != -1))
            self._pddl_interface.addAction(pddl.Action(skill, params, preconds, holdconds, postconds))
        if self._verbose:
            log.info("[Domain]", self._pddl_interface.printDomain(False))

    def get_element(self, uid):
        return self._elements[uid]

    def initProblem(self):
        objects = {}
        elements = {}
        self._elements = {}
        # Find objects
        for objType in self._pddl_interface._types._types["thing"]:
            temp = self._wmi.resolve_elements(wmi.Element(objType))
            elements[objType] = temp
            if len(temp) > 0:
                objects[objType] = []
            for e in temp:
                objects[objType].append(e.id)
                self._elements[e.id] = e
                self._elements[e.id.lower()] = e
        for e in self._abstract_objects:
            ctype = self._wmi.get_super_class(e.type)
            if ctype not in objects:
                objects[ctype] = []
                elements[ctype] = []
            e._id = e.label
            if not e.label in objects[ctype]:  # Avoids duplicates
                objects[ctype].append(e._label)
                elements[ctype].append(e)
                self._elements[e.id] = e
                self._elements[e.id.lower()] = e
        self._pddl_interface.setObjects(objects)
        # Evaluate inital state
        for supertype, types in self._pddl_interface._types._types.iteritems():
            elements[supertype] = []
            for t in types:
                elements[supertype] += elements[t]

        params = skirosp.ParamHandler()
        params.addParam("x", Element(), skirosp.ParamTypes.Required)
        params.addParam("y", Element(), skirosp.ParamTypes.Required)
        for p in self._pddl_interface._predicates:
            if self._wmi.get_reasoner(p.name) is not None:
                # The predicate is handled by a reasoner
                xtype = p.params[0]["valueType"]
                ytype = p.params[1]["valueType"]
                for xe in elements[xtype]:
                    for ye in elements[ytype]:
                        relations = self._wmi.get_reasoner(p.name).computeRelations(xe, ye)
                        if self._verbose:
                            log.info("[Checking {}-{}-{}]".format(xe.id, p.name, ye.id), " Got: {}".format(relations))
                        if p.name in relations:
                            self._pddl_interface.addInitState(pddl.GroundPredicate(p.name, [xe.id, ye.id]))
            else:
                # The predicate is handled normally
                if len(p.params) == 1:
                    if p.value != None:
                        c = cond.ConditionProperty("", p.name, "x", p.operator, p.value, True)
                    else:
                        c = cond.ConditionHasProperty("", p.name, "x", True)
                    xtype = p.params[0]["valueType"]
                    for xe in elements[xtype]:
                        params.specify("x", xe)
                        if c.evaluate(params, self._wmi):
                            self._pddl_interface.addInitState(
                                pddl.GroundPredicate(p.name, [xe._id], p.operator, p.value))
                else:
                    xtype = p.params[0]["valueType"]
                    ytype = p.params[1]["valueType"]
                    subx = [xtype] if self._pddl_interface.getSubTypes(
                        xtype) is None else self._pddl_interface.getSubTypes(xtype)
                    suby = [ytype] if self._pddl_interface.getSubTypes(
                        ytype) is None else self._pddl_interface.getSubTypes(ytype)
                    if p.abstracts:
                        query_str_template = """
                            SELECT ?x ?y WHERE {{
                                    {{ ?xtypes rdfs:subClassOf* {xtype}. }} UNION {{ {xtype} rdfs:subClassOf* ?xtypes. }}
                                    {{ ?ytypes rdfs:subClassOf* {ytype}. }} UNION {{ {ytype} rdfs:subClassOf* ?ytypes. }}
                                    ?xtypes rdfs:subClassOf ?restriction . ?restriction owl:onProperty {relation}. ?restriction ?quantity ?ytypes.
                                    ?x rdf:type/rdfs:subClassOf* ?xtypes. ?y rdf:type/rdfs:subClassOf* ?ytypes.
                            }}"""
                    else:
                        query_str_template = """
                            SELECT ?x ?y WHERE {{
                            {{ ?x {relation} ?y. ?x rdf:type/rdfs:subClassOf* {xtype}. ?y rdf:type/rdfs:subClassOf* {ytype}.}}
                            UNION
                            {{?t {relation} ?z. ?t rdf:type/rdfs:subClassOf* {xtype}. ?z rdf:type/rdfs:subClassOf* {ytype}. ?t skiros:hasTemplate ?x. ?z skiros:hasTemplate ?y. }}
                            UNION
                            {{?t {relation} ?y. ?t rdf:type/rdfs:subClassOf* {xtype}. ?y rdf:type/rdfs:subClassOf* {ytype}. ?t skiros:hasTemplate ?x.}}
                            UNION
                            {{?x {relation} ?z. ?x rdf:type/rdfs:subClassOf* {xtype}. ?z rdf:type/rdfs:subClassOf* {ytype}. ?z skiros:hasTemplate ?y.}}
                            }}"""
                    for x in subx:
                        for y in suby:
                            query_str = query_str_template.format(relation=p.name, xtype=x, ytype=y)
                            answer = self._wmi.query_ontology(query_str)
                            for line in answer:
                                tokens = line.strip().split(" ")
                                self._pddl_interface.addInitState(pddl.GroundPredicate(p.name, tokens))

        for p in self._pddl_interface._functions:
            c = cond.ConditionProperty("", p.name, "x", p.operator, p.value, True)
            xtype = p.params[0]["valueType"]
            for xe in elements[xtype]:
                params.specify("x", xe)
                if c.evaluate(params, self._wmi):
                    self._pddl_interface.addInitState(pddl.GroundPredicate(p.name, [xe._id], p.operator, p.value))
        if self._verbose:
            log.info("[Problem]", self._pddl_interface.printProblem(False))

    def setGoal(self, goal):
        try:
            goals = goal.split(",")
            for g in goals:
                if not g:
                    continue
                if g.find("forall") != -1:
                    self._pddl_interface.addGoal(pddl.ForallPredicate(g))
                else:
                    g = g[1:-1]
                    tokens = g.split(" ")
                    if len(tokens) == 3:
                        self._pddl_interface.addGoal(pddl.GroundPredicate(tokens[0], [tokens[1], tokens[2]]))
                        if tokens[1].find("-") == -1:  # If isAbstractObject
                            self._abstract_objects.append(self._wmi.get_template_element(tokens[1]))
                        if tokens[2].find("-") == -1:  # If isAbstractObject
                            self._abstract_objects.append(self._wmi.get_template_element(tokens[2]))
                    else:
                        self._pddl_interface.addGoal(pddl.GroundPredicate(tokens[0], [tokens[1]]))
                        if tokens[1].find("-") == -1:  # If isAbstractObject
                            self._abstract_objects.append(self._wmi.get_template_element(tokens[1]))
        except BaseException:
            raise Exception("Error while parsing input goal: {}".format(goal))

    def plan(self):
        return self._pddl_interface.invokePlanner()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = TaskManagerNode()
    node.run()
