import skiros2_common.core.params as params
from skiros2_common.core.world_element import Element
import skiros2_common.core.conditions as cond
from skiros2_common.tools.id_generator import IdGen
import skiros2_common.tools.logger as log
from enum import Enum
from copy import copy, deepcopy
from multiprocessing.dummy import Event
from skiros2_common.tools.time_keeper import TimeKeeper

"""
Possible states for a skill
"""
State = Enum('State', 'Success Failure Running Idle Uninitialized')

"""
Unspecify: (postCondition) The parameter is unspecified (removed from the Blackboard) after execution
Consume: (postCondition) The parameter is unspecified and removed from world model after execution
Hardware: (postCondition) The parameter is declared as hardware. The state is set to busy during the skill's execution
RespectType: (preCondition) The value must respect the type of the default parameter.
"""
ParamOptions = Enum('ParamOptions', 'Consume Output Unspecify Lock RespectType')


class SkillDescription(object):
    def __init__(self):
        """
        @brief      An abstract skill description
        """
        # Description
        self._type = "skiros:" + self.__class__.__name__
        self._available_for_planning = True
        # Params
        self._params = params.ParamHandler()
        # Conditions
        self._pre_conditions = []
        self._hold_conditions = []
        self._post_conditions = []
        self.createDescription()
        self.generateDefParams()
        # self.generateDefConditions()

    @property
    def params(self)->params.ParamHandler:
        """
        @brief      Gets the skill's parameter handler
        """
        return self._params

    @property
    def label(self):
        """
        @brief      Gets the skill's label
        """
        return self._label

    @property
    def type(self):
        """
        @brief      Gets the skill's type
        """
        return self._type

    @property
    def available_for_planning(self):
        return self._available_for_planning

    @property
    def wmi(self):
        """
        @brief      Returns the world model interface
        """
        return self._wmi

    def getDescription(self):
        """
        @brief Extracts the skill description
        """
        return (self._type, self._params, self._pre_conditions, self._hold_conditions, self._post_conditions)

    def setDescription(self, typein, paramsin, prein, holdin, postin):
        """
        @brief Sets the skill description (makes a deepcopy)
        """
        self._type = copy(typein)
        self._params = deepcopy(paramsin)
        self._pre_conditions = deepcopy(prein)
        self._hold_conditions = deepcopy(holdin)
        self._post_conditions = deepcopy(postin)

    def addParam(self, key, value, param_type, options=[], description=""):
        """
        @brief Adds a parameter

        key: a unique string identifier
        value: the default value or type
        param_type: the type of parameter (see ParamTypes)
        options: see ParamOptions
        description: an optional verbose description
        """
        self._params.addParam(key, value, param_type, description)
        if isinstance(value, type(Element())):
            for o in options:
                if o == ParamOptions.Consume:
                    self._post_conditions += [self.getGenerateCond("Consume" + key, key, False)]
                elif o == ParamOptions.Unspecify:
                    self._post_conditions += [self.getIsSpecifiedCond("Unset" + key, key, False)]
                elif o == ParamOptions.Lock:
                    self._pre_conditions += [self.getPropCond(key + 'Idle',
                                                              "skiros:StateProperty", key, "=", "Idle", True)]
                    self._hold_conditions += [self.getPropCond(key + 'Busy',
                                                               "skiros:StateProperty", key, "=", "Idle", False)]
                    self._post_conditions += [self.getPropCond(key + 'Idle',
                                                               "skiros:StateProperty", key, "=", "Idle", True)]
                elif o == ParamOptions.RespectType:
                    self._pre_conditions.append(self.getOnTypeCond(key + 'OfType', key, self.params[key].default.type))

    def setAvailableForPlanning(self, available=True):
        """
        @brief Specify if the skill can be used for planning
        """
        self._available_for_planning = available

    def generateDefParams(self):
        """
        @brief Some default params are added automatically
        """
        if not self._params.hasParam('Robot'):
            self._params.addParam("Robot", Element("sumo:Agent"), params.ParamTypes.Inferred)
        # if not self._params.hasParam('Skill'):
        #    self._params.addParam("Skill", self.toElement(), params.ParamTypes.Required)

    def generateDefConditions(self):
        """
        @brief Some default preconditions are added automatically
        """
        #self.addPreCondition(self.getRelationCond("HasSkill", "hasSkill", "Robot", "Skill", True))
        # for key, param in self._params.getParamMapFiltered(params.ParamTypes.Hardware).iteritems():
        #    self.addPreCondition(self.getPropCond("DeviceIdle", "deviceState", key, "Idle", True))
        for key, param in self._params.getParamMapFiltered(params.ParamTypes.Optional).items():
            if isinstance(Element(), param.dataType()):
                c1 = self.getGenerateCond("Has" + key, key, True)
                dont_add = False
                for c2 in self._post_conditions:
                    if c1.isEqual(c2) or c1.hasConflict(c2):
                        dont_add = True
                if not dont_add:
                    self._post_conditions = [c1] + self._post_conditions
        return True

    def getOutputParams(self):
        return self._params.getParamMapFiltered(params.ParamTypes.Optional)

    def addPreCondition(self, condition):
        self._pre_conditions.append(condition)

    def addHoldCondition(self, condition):
        self._hold_conditions.append(condition)

    def addPostCondition(self, condition):
        self._post_conditions.append(condition)

    def getOrCond(self, desired_state):
        return cond.ConditionOr(desired_state)

    def getIsSpecifiedCond(self, clabel, subj, desired_state):
        return cond.ConditionIsSpecified(clabel, subj, desired_state)

    def getGenerateCond(self, clabel, subj, desired_state):
        return cond.ConditionGenerate(clabel, subj, desired_state)

    def getHasPropCond(self, clabel, olabel, subj, desired_state):
        return cond.ConditionHasProperty(clabel, olabel, subj, desired_state)

    def getPropCond(self, clabel, olabel, subj, operator, value, desired_state):
        return cond.ConditionProperty(clabel, olabel, subj, operator, value, desired_state)

    def getAbstractRelationCond(self, clabel, olabel, subj, obj, desired_state):
        return cond.AbstractConditionRelation(clabel, olabel, subj, obj, desired_state)

    def getRelationCond(self, clabel, olabel, subj, obj, desired_state):
        return cond.ConditionRelation(clabel, olabel, subj, obj, desired_state)

    def getOnTypeCond(self, clabel, subj, value):
        return cond.ConditionOnType(clabel, subj, value)

    def getModifiedParams(self):
        param_list = set([])
        for c in self._post_conditions:
            # if isinstance(c, cond.ConditionGenerate):
            param_list = param_list.union(set(c.getKeys()))
        return param_list

    def printInfo(self, verbose=False):
        s = "{}".format(self._type)
        if verbose:
            s += "["
            s += self._params.printState() + ']\n'
            s += self.printConditions()
        else:
            s += "\n"
        return s

    def printConditions(self):
        s = ""
        if self._pre_conditions:
            s += "PreConditions:\n"
            for c in self._pre_conditions:
                s += '{}\n'.format(c._description)
        if self._hold_conditions:
            s = "HoldConditions:\n"
            for c in self._hold_conditions:
                s += '{}\n'.format(c._description)
        if self._post_conditions:
            s += "PostConditions:\n"
            for c in self._post_conditions:
                s += '{}\n'.format(c._description)
        return s

    def toElement(self):
        to_ret = Element(self._type)
        to_ret._label = self._label
        to_ret.setProperty("skiros:AvailableForPlanning", self.available_for_planning)
        for _, p in self.params.items():
            if p.dataTypeIs(Element):
                to_ret.addRelation(self, "skiros:hasParam", p.toElement())
        for c in self._pre_conditions:
            to_ret.addRelation(self, "skiros:hasPreCondition", c.toElement())
        for c in self._hold_conditions:
            to_ret.addRelation(self, "skiros:hasHoldCondition", c.toElement())
        for c in self._post_conditions:
            to_ret.addRelation(self, "skiros:hasPostCondition", c.toElement())
        return to_ret

    # --------Virtual functions--------
    def createDescription(self):
        """ Optional, Not implemented in abstract class. """
        return


class SkillCore(SkillDescription):
    gen_id = IdGen()

    def __init__(self):
        """
        @brief      An abstract executable skill with a description (type,
                    label, params, conditions), a state and progress code
        """
        # Description
        self._id = SkillCore.gen_id.getId()
        self._type = ""
        self._label = ""
        self._available_for_planning = True
        self._description = SkillDescription()
        # Params
        self._params = params.ParamHandler()
        # Conditions
        self._pre_conditions = []
        self._hold_conditions = []
        self._post_conditions = []
        # Execution
        self._state_change = Event()
        self._state = State.Uninitialized
        self._avg_time_keeper = TimeKeeper()
        self._time_keeper = TimeKeeper()
        self._progress_code = 0
        self._progress_period = 0.0
        self._progress_time = 0.0
        self._progress_msg = ""
        self._expand_on_start = False
    # --------Class functions--------

    def expand(self, skill):
        return

    def hasChildren(self):
        return False

    def _setState(self, state):
        self._state = state
        if type(state) is bool:
            log.error("Expected 'State' enum with 'success', 'step' or 'fail', but got a bool.")
            if state:
                self._state = self.success("Fixme: Set success for wrong return type bool.")
            else:
                self._state = self.fail("Fixme: Set failure for wrong return type bool.", -99)
        self._state_change.set()

    def _setProgress(self, msg, code=None):
        if code is None:
            code = self._progress_code + 1
        self._progress_code = code
        self._progress_period = self._avg_time_keeper.get_avg_time()
        self._progress_time = self._time_keeper.time_from_start()
        self._progress_msg = str(msg)

    @property
    def id(self):
        return self._id

    @property
    def progress_code(self):
        return self._progress_code

    @property
    def progress_period(self):
        return self._progress_period

    @property
    def progress_time(self):
        return self._progress_time

    @property
    def progress_msg(self):
        return self._progress_msg

    @property
    def state(self):
        return self._state

    @property
    def expand_on_start(self):
        """
        @brief      Default False. If true, the skill will expand every time it
                    is started. Used e.g. in a planner skill
        """
        return self._expand_on_start

    def _resetDescription(self, other=None):
        if other:
            self._params.reset(self._description._params.merge(other._params))
        else:
            self._params = deepcopy(self._description._params)
        self._pre_conditions = deepcopy(self._description._pre_conditions)
        self._hold_conditions = deepcopy(self._description._hold_conditions)
        self._post_conditions = deepcopy(self._description._post_conditions)

    def hasPreCond(self):
        return bool(self._pre_conditions)

    def checkPreCond(self, verbose=False):
        """
        @brief      Check pre-conditions.

        @param      verbose  (bool) Print error message when check fail

        @return     A list of parameters that breaks the conditions, or an empty
                    list if all are satisfied
        """
        to_ret = list()
        err_msg = ""
        for c in self._pre_conditions:
            if not c.evaluate(self._params, self._wmi):
                err_msg += "{} Check failed. \n".format(c.getDescription())
                if verbose:
                    log.error(c.getDescription(), "ConditionCheck failed")
                to_ret += c.getKeys()
        self._setProgress(err_msg, -1)
        return list(set(to_ret))

    def checkHoldCond(self, verbose=False):
        """
        @brief      Check hold-conditions.

        @param      verbose  (bool) Print error message when check fail

        @return     A list of parameters that breaks the conditions, or an empty
                    list if all are satisfied
        """
        to_ret = list()
        err_msg = ""
        for c in self._hold_conditions:
            if not c.evaluate(self._params, self._wmi):
                err_msg += "{} Check failed. \n".format(c.getDescription())
                if verbose:
                    log.error("HoldConditionCheck failed", c.getDescription())
                to_ret += c.getKeys()
        self._setProgress(err_msg, -2)
        return list(set(to_ret))

    def hasPostCond(self):
        return bool(self._post_conditions)

    def checkPostCond(self, verbose=False):
        """
        @brief      Check post-conditions.

        @param      verbose  (bool) Print error message when check fail

        @return     A list of parameters that breaks the conditions, or an empty
                    list if all are satisfied
        """
        to_ret = list()
        for c in self._post_conditions:
            if not c.evaluate(self._params, self._wmi):
                if verbose:
                    log.error(c.getDescription(), "ConditionCheck failed")
                to_ret += c.getKeys()
        return list(set(to_ret))
    # -------- Control functions--------

    def preempt(self):
        if self.hasState(State.Running):
            self._setState(self.onPreempt())
            if not self.onEnd():
                self._setState(State.Failure)
        return self._state

    def getState(self):
        return self._state

    def hasState(self, state):
        return self._state == state

    def waitState(self, state, isset=True):
        if isset:  # Xor?
            while self._state != state:
                # print 'Waiting set.. {}'.format(self._state)
                self._state_change.clear()
                self._state_change.wait()
        else:
            while self._state == state:
                # print 'Waiting not set.. {}'.format(self._state)
                self._state_change.clear()
                self._state_change.wait()
        # print 'State changed {}'.format(self._state)

    def reset(self):
        self.onReset()
        self._params.setDefault()
        self._time_keeper.reset()
        self._avg_time_keeper.reset()
        self._setProgress("", 0)
        self._setState(State.Idle)
        return self._state

    def start(self, params=None):
        if params:
            self.specifyParams(params, False)
        self._time_keeper.reset()
        if self.onStart():
            self._setState(State.Running)
            self._setProgress("Start", 0)
        else:
            log.warn("start", "onStart function of skill {} did not succeed.".format(self.label))
            self._setState(State.Failure)
        return self._state

    def printInfo(self, verbose=False):
        s = "{}-{} ".format(self._type, self._label)
        if verbose:
            s += "["
            s += self._params.printState() + ']\n'
            s += self.printConditions()
        else:
            s += "\n"
        return s

    def printState(self, verbose=False):
        s = "{}-{}({})".format(self.type[self.type.find(":") + 1:], self.label, self.state)
        if verbose:
            s += "[{}]".format(self.params.printState())
        return s

    def printProgress(self):
        return "[{}] {}".format(self._progress_code, self._progress_msg)

    def specifyParamDefault(self, key, values):
        """
        @brief      Specify a value and set it as default value too

        @param      key     (string) Parameter key
        @param      values  Parameter value(s)
        """
        if not self._params.hasParam(key):
            log.error("specifyParamDefault", "No param '{}' found. Debug: {}".format(key, self.printInfo(True)))
        self._params.specifyDefault(key, values)

    def specifyParam(self, key, values):
        """
        @brief      Specify a parameter and update the input cache

        @param      key     (string) Parameter key
        @param      values  Parameter value(s)
        """
        if not self._params.hasParam(key):
            log.error("specifyParam", "No param '{}' found. Debug: {}".format(key, self.printInfo(True)))
        self._params.specify(key, values)

    def specifyParamsDefault(self, input_params):
        """
        @brief      Set the parameters and makes them default (they will no more
                    be overwritten by specifyParams, even with
                    keep_offline=False)

        @param      input_params  (dict)
        """
        self._params.specifyParamsDefault(input_params)

    def specifyParams(self, input_params, keep_default=True):
        """
        @brief      Set the parameters

        @param      input_params  (dict) Parameters to set
        @param      keep_default  (bool) If True, params already specified are
                                  preserved
        """
        self._params.specifyParams(input_params, keep_default)

    # -------- User's functions--------
    def setDescription(self, description, label=""):
        """
        @brief Description is a SkillDescription
        """
        self._description = description
        self._type = description._type
        if label != "":
            self._label = label
        self._resetDescription()

    def startError(self, msg, code):
        """
        @brief signal an error during the starting routine
        """
        assert type(msg) == str
        assert type(code) == int
        self.fail(msg, code)
        return False

    def step(self, msg=""):
        """
        @brief Set a running breakpoint
        """
        assert type(msg) == str
        self._setProgress(msg)
        return State.Running

    def fail(self, msg, code):
        """
        @brief Set a failure state
        """
        assert type(msg) == str
        assert type(code) == int
        if code > 0:
            code *= -1
        self._setProgress(msg, code)
        return State.Failure

    def success(self, msg=""):
        """
        @brief Set a success state
        """
        assert type(msg) == str
        self._setProgress(msg)
        return State.Success

    # -------- Virtual functions--------
    def modifyDescription(self, skill):
        """
        @brief Override to define additional parameters/condition over the skill
        """
        pass

    def onReset(self):
        """
        @brief Called when resetting.
        """
        pass

    def onStart(self):
        """
        @brief Called just before 1st execute

        @return (Bool)
        """
        return True

    def onPreempt(self):
        """
        @brief Called when skill is requested to stop.

        @return (State)
        """
        self._setProgress("Preempted", -1)
        return State.Failure

    def execute(self):
        """
        @brief Main execution function

        @return (State)
        """
        raise NotImplementedError("Not implemented in abstract class")

    def onEnd(self):
        """
        @brief Called just after last execute or preemption

        @return (Bool)
        """
        return True
