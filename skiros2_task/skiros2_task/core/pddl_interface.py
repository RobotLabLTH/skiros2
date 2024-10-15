import subprocess
import os
import skiros2_common.tools.logger as log

try:
    basestring
except NameError:
    basestring = str
    
class PddlTypes(object):
    __slots__ = '_types'

    def __init__(self):
        self._types = {}

    def addType(self, name, supertype):
        if name == supertype:
            return
        if supertype not in self._types:
            self._types[supertype] = []
        if not name in self._types[supertype]:
            self._types[supertype].append(name)

    def toPddl(self):
        string = "(:types \n"
        for supertype, types in self._types.items():
            string += '\t'
            string += ' '.join(types)
            string += " - {}\n".format(supertype)
        string += ")"
        return string


class Predicate(object):
    __slots__ = 'name', 'params', 'negated', 'operator', 'value', 'abstracts'

    def __eq__(self, other):
        if self.name != other.name:
            return False
        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def isEqualOf(self, other):
        if self.name == other.name and len(self.params) == len(other.params):
            for p1, p2 in zip(self.params, other.params):
                if not p1["key"] == p2["key"]:
                    return False
            if self.negated == other.negated:
                return True
        return False

    def isNegatedOf(self, other):
        if self.name == other.name and len(self.params) == len(other.params):
            for p1, p2 in zip(self.params, other.params):
                if not p1["key"] == p2["key"]:
                    return False
            if self.negated != other.negated:
                return True
        return False

    def __init__(self, predicate, params, abstracts):
        self.name = predicate.getProperty("skiros:appliedOnType").value
        self.operator = None
        self.value = None
        self.abstracts = abstracts
        self.negated = not predicate.getProperty("skiros:desiredState").value
        self.params = []
        sub = predicate.getProperty("skiros:hasSubject").value
        self.params.append({"paramType": "x", "key": sub, "valueType": params[sub]})
        if predicate.hasProperty("skiros:hasObject"):
            obj = predicate.getProperty("skiros:hasObject").value
            self.params.append({"paramType": "y", "key": obj, "valueType": params[obj]})
        if predicate.hasProperty("skiros:operator"):
            self.operator = predicate.getProperty("skiros:operator").value
            self.value = predicate.getProperty("skiros:desiredValue").value

    def isFunction(self):
        return self.operator is not None and not isinstance(self.value, str) and not isinstance(self.value, bool)
    
    def value_is_bool(self):
        return isinstance(self.value, bool) or self.value in ["true", "false", "True", "False"]
    
    def property_string(self):
        if self.value_is_bool():
            return self.name
        if ":" in self.name:
            prop_str = self.name.split(":", 1)[1]
        else:
            prop_str = self.name
        return f"{prop_str}_{str(self.value)}"
    
    def is_negated(self, invert):
        negate = self.negated or (not self.negated and invert)
        if self.value_is_bool() and (not self.value or self.value in ["false", "False"]):
            return not negate
        return negate

    def toActionPddl(self, invert=False):
        string = ''
        if self.is_negated(invert):
            string += '(not '
        if self.isFunction():
            string += '({} '.format(self.operator)
        if isinstance(self.value, basestring) or isinstance(self.value, bool):
            string += '({}'.format(self.property_string())
        else:
            string += '({}'.format(self.name)
        for p in self.params:
            string += ' ?{}'.format(p["key"])
        if self.isFunction():
            string += ') {}'.format(self.value)
        if self.is_negated(invert):
            string += ')'
        string += ")"
        return string

    def toUngroundPddl(self):
        if isinstance(self.value, str) or isinstance(self.value, bool):
            string = '({}'.format(self.property_string())
        else:
            string = '({}'.format(self.name)
        for p in self.params:
            string += ' ?{} - {} '.format(p["paramType"], p["valueType"])
        string += ")"
        return string


class GroundPredicate(object):
    __slots__ = 'name', 'params', 'operator', 'value'

    def __init__(self, name, params, operator=None, value=None):
        self.name = name
        self.params = params
        self.operator = operator
        self.value = value

    def __eq__(self, other):
        # TODO: check with the functions..
        if self.name == other.name and len(self.params) == len(other.params):
            return set(self.params) == set(other.params)
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def isFunction(self):
        return self.operator is not None and not isinstance(self.value, str) and not isinstance(self.value, bool)

    def value_is_bool(self):
        return isinstance(self.value, bool) or self.value in ["true", "false", "True", "False"]

    def property_string(self):
        if self.value_is_bool():
            return self.name
        if ":" in self.name:
            prop_str = self.name.split(":", 1)[1]
        else:
            prop_str = self.name
        return f"{prop_str}_{str(self.value)}"

    def is_negated(self):
        if self.value_is_bool() and (not self.value or self.value in ["false", "False"]):
            return True
        return False

    def toPddl(self):
        string = ''
        if self.is_negated():
            string += '(not '
        if self.isFunction():
            string += '({} '.format(self.operator)
        if isinstance(self.value, basestring) or isinstance(self.value, bool):
            string += '({}'.format(self.property_string())
        else:
            string += '({}'.format(self.name)
        for p in self.params:
            string += ' {}'.format(p)
        if self.isFunction():
            string += ') {}'.format(self.value)
        if self.is_negated():
            string += ')'
        string += ")"
        return string


class ForallPredicate(object):
    __slots__ = 'predicate'

    def __init__(self, predicate):
        self.predicate = predicate

    def toPddl(self):
        return self.predicate


class Action(object):
    __slots__ = 'label', 'name', 'params', 'preconditions', 'holdconditions', 'effects'

    def __init__(self, skill, params, precons, holdcons, postcons):
        self.label = skill._label 
        self.name = self.label.replace(" ", "_").lower()  # tfd can not handle spaces
        self.params = params
        self.holdconditions = holdcons
        self.preconditions = precons
        self.effects = postcons

    def __eq__(self, other):
        if self.name != other.name:
            return False
        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def toPddl(self):
        string = '(:durative-action {}\n'.format(self.name)
        string += "\t:parameters ("
        for p, t in self.params.items():
            string += "?{} - {} ".format(p, t)
        string += ")\n"
        string += '\t:duration (= ?duration 1)\n'
        string += '\t:condition (and\n'
        for p in self.preconditions:
            string += '\t\t(at start {})\n'.format(p.toActionPddl())
        for p in self.holdconditions:
            string += '\t\t(over all {})\n'.format(p.toActionPddl())
        string += "\t)\n"
        string += "\t:effect (and\n"
        for e in self.effects:
            application_time = "end"
            for p in self.preconditions:
                if e.isNegatedOf(p):
                    application_time = "start"
                    break
                elif e.isEqualOf(p):
                    string += '\t\t(at {} {})\n'.format("start", e.toActionPddl(invert=True))
                    break
            string += '\t\t(at {} {})\n'.format(application_time, e.toActionPddl())
        string += "\t)\n"
        string += ")\n"
        return string


class PddlInterface:
    def __init__(self, workspace="~/.skiros/planner", title="untitled"):
        """
        @brief      Class to manage a pddl domain and do task planning

                    It generates a pddl definition and invokes a task planner

        @param      workspace  (string) The workspace folder were pddl files are generated
        @param      title      (string) Title for the pddl domain
        """
        self._title = title
        self._workspace = os.path.expanduser(workspace)
        if not os.path.exists(self._workspace):
            os.makedirs(self._workspace)
        self.clear()

    def clear(self):
        self._types = PddlTypes()
        self._objects = {}
        self._functions = []
        self._predicates = []
        self._actions = dict()
        self._init_state = []
        self._goal = []

    def getSubTypes(self, supertype):
        if supertype in self._types._types:
            return self._types._types[supertype]

    def getAction(self, name):
        return self._actions[name] if name in self._actions else None

    def _addSuperTypes(self, predicate):
        lookuplist = self._predicates
        if predicate.isFunction():
            lookuplist = self._functions
        for p in lookuplist:
            if p == predicate:
                for param1, param2 in zip(p.params, predicate.params):
                    if param1["valueType"] != param2["valueType"]:
                        supertypeId = p.name + param1["paramType"]
                        self._types.addType(param1["valueType"], supertypeId)
                        self._types.addType(param2["valueType"], supertypeId)
                        param1["valueType"] = supertypeId
                return

    def addType(self, name, supertype):
        self._types.addType(name, supertype)

    def addUngroundPredicate(self, predicate):
        if predicate.isFunction():
            self.addFunction(predicate)
            return
        if not predicate in self._predicates:
            self._predicates.append(predicate)
        else:
            self._addSuperTypes(predicate)

    def addFunction(self, function):
        if not function in self._functions:
            self._functions.append(function)
        else:
            self._addSuperTypes(function)

    def addAction(self, action):
        if not action in self._actions.values() and action.preconditions and action.effects:
            for p in action.params.values():
                self.addType(p, "thing")
            for c in action.preconditions:
                self.addUngroundPredicate(c)
            for c in action.effects:
                self.addUngroundPredicate(c)
            self._actions[action.name] = action

    def setObjects(self, objects):
        self._objects = objects

    def addInitState(self, state):
        for s in self._init_state:
            if s == state:
                return
        self._init_state.append(state)

    def addGoal(self, g):
        self._goal.append(g)

    def printDomain(self, to_file=False):
        string = "(define (domain {})\n".format(self._title)
        string += "(:requirements :typing :fluents :universal-preconditions)\n"  # TODO: make this dynamic?
        string += self._types.toPddl()
        string += "\n"
        string += "(:predicates \n"
        for p in self._predicates:
            string += "\t"
            string += p.toUngroundPddl()
            string += "\n"
        string += ")\n"
        string += "(:functions \n"
        for f in self._functions:
            string += "\t{}\n".format(f.toUngroundPddl())
        string += ")\n"
        for a in self._actions.values():
            string += a.toPddl()
            string += "\n"
        string += ")\n"
        if to_file:
            with open(self._workspace + "/domain.pddl", 'w') as f:
                f.write(string)
        else:
            return string

    def printProblem(self, to_file=False):
        string = "(define (problem {}) (:domain {})\n".format("1", self._title)
        string += "(:objects \n"
        for objType, objects in self._objects.items():
            if len(objects):
                string += '\t'
                string += ' '.join(objects)
                string += ' - {}\n'.format(objType)
        string += ")\n"
        string += "(:init \n"
        for state in self._init_state:
            string += "\t"
            string += state.toPddl()
            string += "\n"
        string += ")\n"
        string += "(:goal (and \n"
        for g in self._goal:
            string += '\t'
            string += g.toPddl()
            string += "\n"
        string += "))\n"
        string += ")\n"
        if to_file:
            with open(self._workspace + "/p01.pddl", 'w') as f:
                f.write(string)
        else:
            return string

    def selectMinDurationPlan(self, outpaths):
        """
        @brief Selects the shortest plan
        """
        data = []
        for p in outpaths:
            with open(p, 'r') as f:
                data.append(f.read())
            os.remove(p)
        return min(data, key=len)#TODO: implement a proper selection


    def invokePlanner(self, generate_pddl=True):
        """
        @brief      Generate pddl files and invoke TFD planner

        @param      generate_pddl  (bool) If False uses previously generated
                                   pddl files

        @return     (string) The plan
        """
        if generate_pddl:
            self.printDomain(True)
            self.printProblem(True)

        config = "y+Y+a+T+10+t+5+e+r+O+1+C+1"
        result_name = self._workspace + "/pddlplan"
        path = os.environ.get('TFD_HOME')

        #Copied from tfd-src-0.4/plan.py
        # run translator
        self.run(path + "/translate/translate.py", self._workspace + "/domain.pddl", self._workspace + "/p01.pddl")
        # run preprocessing
        self.run(path + "/preprocess/preprocess", input="output.sas")
        # run search
        self.run(path + "/search/search", config, "p", result_name, input="output")

        outpaths = []
        for (dirpath, dirnames, filenames) in os.walk(self._workspace):
            for name in filenames:
                if name.find('pddlplan')>=0:
                    outpaths.append(dirpath+'/'+name)
        if outpaths:
            data = self.selectMinDurationPlan(outpaths)
            try:
                os.remove("output")
                os.remove("all.groups")
                os.remove("variables.groups")
                os.remove("output.sas")
            except BaseException:
                log.warn("[TOCHECK]", "Not all files were generated while planning.")
            return data
        else:
            return None

    def getActionParamMap(self, name, values):
        """
        @brief Returns a key-value map from a planned action
        """
        a = self._actions[name]
        to_ret = dict()
        for k, v in zip(a.params.keys(), values):
            to_ret[k] = v
        return to_ret

    def run(self, *args, **kwargs):
        """
        @brief      Copied from tfd-src-0.4/plan.py. Redirects stdin and stdout
                    and invokes a script

        @param      args    The arguments
        @param      kwargs  The keywords arguments
        """
        input = kwargs.pop("input", None)
        assert not kwargs
        redirections = {}
        if input:
            redirections["stdin"] = open(input)
        redirections["stdout"] = subprocess.PIPE
        log.debug("[run]", "{} {}".format(args, redirections))
        output = subprocess.Popen(sum([arg.split("+") for arg in args],[]), **redirections).communicate()[0]
        log.debug("[output]", str(output))
