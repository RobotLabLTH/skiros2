import skiros2_common.core.params as params
import skiros2_common.tools.logger as log
from .processors import *
from copy import deepcopy
from copy import copy
from collections import OrderedDict, defaultdict
from skiros2_common.core.abstract_skill import SkillDescription, SkillCore, State, ParamOptions


class SkillPreempted(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class SkillInterface(SkillCore):
    """
    """
    #--------Class functions--------

    def __init__(self, children_processor=Sequential()):
        super(SkillInterface, self).__init__()
        # Params
        self._remaps = OrderedDict()
        # Connections
        self._parent = None
        self._children = []
        self._children_processor = children_processor
        # Caches
        self._was_simulated = False
        self._max_cache = 2
        self._params_cache = []
        self._input_cache = []
        self._remaps_cache = defaultdict(list)

    def __call__(self, *children):
        """
        @brief Add a set of children skills
        @return self for nested children declarations
        """
        for c in children:
            self.addChild(c)
        return self

    @property
    def parent(self):
        """
        @brief      Returns the parent skill
        """
        return self._parent

    def getLightCopy(self):
        """
        @brief      Makes a light copy (only description, params and state)

        @return     A light copy
        """
        p = self.__class__(self._children_processor)
        p._children_processor = deepcopy(self._children_processor)
        p._type = copy(self._type)
        p._label = copy(self._label)
        p._available_for_planning = self._available_for_planning
        p._params = deepcopy(self._params)
        p._remaps = deepcopy(self._remaps)
        p._description = deepcopy(self._description)
        p._pre_conditions = deepcopy(self._pre_conditions)
        p._hold_conditions = deepcopy(self._hold_conditions)
        p._post_conditions = deepcopy(self._post_conditions)
        if self._state != State.Uninitialized:
            p._state = copy(self._state)
            p._wmi = self._wmi
        return p

    def _setState(self, state):
        self._state = state
        self._state_change.set()

    def _clearRemaps(self):
        """
        @brief      Clear remaps
        """
        for r1, r2 in reversed(list(self._remaps.items())):
            self.remap(r2, r1)
        self._remaps = OrderedDict()
        self._remaps_cache = defaultdict(list)

    def _copyRemaps(self, skill):
        """
        @brief      Copy the remaps of another skill. Called automatically when
                    the skill is added as a child
        """
        for r1, r2 in skill._remaps.items():
            self.remap(r1, r2)

    def _revertRemaps(self):
        """
        @brief      Revert remaps. Just used in revertInput
        """
        remapid = len(self._params_cache)
        try:
            if self._remaps_cache[remapid]:
                for remap in self._remaps_cache[remapid]:
                    log.warn("REMAP", "Revert {} {}".format(remap[0], remap[1]))
                    print(self._remaps_cache)
                    self._remaps.pop(remap[0])
                    self.remap(remap[1], remap[0], False)
                del self._remaps_cache[remapid]
        except BaseException:
            print(self.printInfo(True))
            print(self._remaps_cache)
            raise

    def getParamsNoRemaps(self):
        """
        @brief Get the skill's parameters without key remappings
        """
        ph = params.ParamHandler()
        ph.reset(self._params.getCopy())
        for r1, r2 in reversed(list(self._remaps.items())):
            ph.remap(r2, r1)
        return ph

    def get_remap(self, key):
        """
        @brief Return the key remapping if existing or otherwise the key unchanged
        """
        return self._remaps[key] if key in self._remaps else key

    def remap(self, initial_key, target_key, record=True):
        """
        @brief Remap a parameter with initial_key to a new target_key

        All skill's children are remapped too.
        """
        # Ignore harmful remappings
        if initial_key in self._remaps:
            if self._remaps[initial_key] == target_key:  # Redundant
                #log.warn(self.type, "Ignoring redundant remap {}->{}".format(initial_key, target_key))
                return
            else:  # Already remapped
                log.warn(self.type, "Key {} already remapped to {}. Can t remap to {}".format(initial_key, self._remaps[initial_key], target_key))
                return

        #log.warn("Remap", "{} {} {}. Existing: {}".format(self.type, initial_key, target_key, self._remaps))
        # The current 2->3 is related to an existing remap 1->2
#        if initial_key in self._remaps.values():
#            log.info("ChainRemap", "{}: {}->{}->{}".format(self.type, self._remaps.keys()[self._remaps.values().index(initial_key)], initial_key, target_key))

        for c in self._children:
            c.remap(initial_key, target_key)

        if target_key in self._remaps:
            # Asking remap 1->2 but exists a remap 2->3. Records a remap 1->3
            target_key = self.get_remap(target_key)

        if self.params.hasParam(target_key):
            log.error("Remap", "{}: Invalid remapping {}->{}, target key is already present.".format(self.type, initial_key, target_key))
            return

        if self.params.hasParam(initial_key):
            #log.warn("{}".format(self.type), "REMAPPING: {} {}".format(initial_key, target_key))
            # Remaps
            self._params.remap(initial_key, target_key)
            for c in self._pre_conditions:
                c.remap(initial_key, target_key)
            for c in self._hold_conditions:
                c.remap(initial_key, target_key)
            for c in self._post_conditions:
                c.remap(initial_key, target_key)
        # Records
        if record:
            self._remaps[initial_key] = target_key
            remapid = len(self._params_cache)
            if remapid > 0:  # If I have params cached, I cache also the remap to revert it afterwards
                self._remaps_cache[remapid].append((initial_key, target_key))

    def init(self, wmi):
        self._wmi = wmi
        self.createDescription()
        self.modifyDescription(self)
        self._setState(State.Idle)

    def inSubtreeOf(self, skill):
        if self in skill._children:
            return True
        for c in skill._children:
            if self.inSubtreeOf(c):
                return True
        return False

    def hold(self):
        for c in self._hold_conditions:
            if not c.setTrue(self._params, self._wmi):
                log.error(c.getDescription(), "Hold failed.")
                return False
        return True

    def revertHold(self):
        for c in reversed(self._hold_conditions):
            #print c.getDescription()
            if not c.revert(self._params, self._wmi):
                log.error(c.getDescription(), "Revert hold failed.")
                return False
        self._was_simulated = False
        return True

    def simulate(self):
        self._was_simulated = True
        #print "Simulate: {}".format(self.printInfo(True))
        for c in self._post_conditions:
            if not c.setTrue(self._params, self._wmi):
                log.error(c.getDescription(), "Simulation failed.")
                return State.Failure
        return State.Success

    def revertSimulation(self):
        if not self._was_simulated:
            log.warn("revert", "No simulation was made, can't revert.")
            return False
        for c in reversed(self._post_conditions):
            if not c.revert(self._params, self._wmi):
                log.error(c.getDescription(), "Revert failed.")
                return False
        self._was_simulated = False
        return True

    def visit(self, visitor):
        return visitor.process(self)

    def visitPreempt(self, visitor):
        return visitor.processPreempt(self)

    def getParent(self):
        return self._parent

    def hasChildren(self):
        return len(self._children) > 0

    def addChild(self, p, latch=False):
        p._parent = self
        self._children.append(p)
        p._copyRemaps(self)
        if latch and len(self._children) > 1:
            for c in self._children[-2]._post_conditions:
                for key in c.getKeys():
                    if not p.params.hasParam(key):
                        p.params[key] = deepcopy(self._children[-2]._params._params[key])
            p._pre_conditions += deepcopy(self._children[-2]._post_conditions)
        return self

    def last(self):
        return self._children[-1]

    def popChild(self):
        child = self._children.pop()
        child._parent = None

    def specifyParam(self, key, values):
        """
        Specify a parameter and update the input cache
        """
        super(SkillInterface, self).specifyParam(key, values)
        if self._input_cache:
            if key in self._input_cache[-1]:
                self._input_cache[-1][key].setValues(values)

    def specifyParamsDefault(self, input_params):
        """
        Set the parameters and makes them default (they will no more be overwritten by setInput)
        """
        self._params_cache.append(self._params.getCopy())
        self._input_cache.append(input_params.getCopy())
        super(SkillInterface, self).specifyParamsDefault(input_params)

    def specifyParams(self, input_params, keep_default=True):
        """
        Set the parameters. Params already specified are preserved
        """
        self._params_cache.append(self._params.getCopy())
        self._input_cache.append(input_params.getCopy())
        super(SkillInterface, self).specifyParams(input_params, keep_default)
        # Erase the oldest cache if the max lenght is reached
        if len(self._params_cache) > self._max_cache:
            self._popCache()

    def _popCache(self):
        self._params_cache.pop(0)
        for i in range(self._max_cache + 1):
            if i not in self._remaps_cache:
                continue
            del self._remaps_cache[i]
            if i + 1 in self._remaps_cache:
                self._remaps_cache[i] = self._remaps_cache[i + 1]

    def revertInput(self):
        if not self._params_cache:
            log.warn("revertInput", "No cache available, can't revert input.")
            return None
        self._revertRemaps()
        self._params.reset(deepcopy(self._params_cache.pop()))
        return deepcopy(self._input_cache.pop())

    def start(self, params=None):
        self._children_processor.reset()
        return SkillCore.start(self, params)

    def tick(self):
        res = self.execute()
#        print "{} {}".format(self, res)
        if res is not None:
            self._setState(res)
        if not self.onEnd():
            self._setState(State.Failure)
            res = State.Failure
        return res

    #--------User functions--------
    def resetDescription(self, other=None):
        if other:
            self._params.reset(self._description._params.merge(other._params))
        else:
            self._params = deepcopy(self._description._params)
        self._pre_conditions = list()
        self._hold_conditions = list()
        self._post_conditions = list()
        for p in self._description._pre_conditions:
            self.addPreCondition(deepcopy(p))
        for p in self._description._hold_conditions:
            self.addHoldCondition(deepcopy(p))
        for p in self._description._post_conditions:
            self.addPostCondition(deepcopy(p))

    def mergeDescription(self, other):
        self.resetDescription(other)
        for c in other._pre_conditions:
            if not c in self._pre_conditions:
                self.addPreCondition(c)
        for c in other._hold_conditions:
            if not c in self._hold_conditions:
                self.addHoldCondition(c)
        for c in other._post_conditions:
            if not c in self._post_conditions:
                self.addPostCondition(c)

    def addParam(self, key, value, param_type, options=[], description=""):
        """
        @brief Check for remaps before adding a parameter.

        See SkillDescription for more details
        """
        if key in self._remaps:
            key = self._remaps[key]
        SkillDescription.addParam(self, key, value, param_type, options, description)

    def addPreCondition(self, condition, modify_description=False):
        if modify_description:
            self._description.addPreCondition(deepcopy(condition))
        self._pre_conditions.append(condition)
        for r1, r2 in self._remaps.items():
            self._pre_conditions[-1].remap(r1, r2)

    def addHoldCondition(self, condition, modify_description=False):
        if modify_description:
            self._description.addHoldCondition(deepcopy(condition))
        self._hold_conditions.append(condition)
        for r1, r2 in self._remaps.items():
            self._hold_conditions[-1].remap(r1, r2)

    def addPostCondition(self, condition, modify_description=False):
        if modify_description:
            self._description.addPostCondition(deepcopy(condition))
        self._post_conditions.append(condition)
        for r1, r2 in self._remaps.items():
            self._post_conditions[-1].remap(r1, r2)

    def processChildren(self, visitor):
        """
        TODO: this function has to be removed and embedded into the tick
        """
        self._setState(self._children_processor.processChildren(self._children, visitor))
        return self._state

    def setProcessor(self, processor_type):
        """
        Set the children processor
        """
        self._children_processor = processor_type
    #--------Virtual functions--------

    def hasInstance(self):
        """ Wrapper skills override this """
        return True

    def execute(self):
        """
            Optional
        """
        self._setProgress("End", 1)
        return None

    def wrapper_expand(self):
        pass


class SkillWrapper(SkillInterface):
    """
    A skill wrapper is a placeholder for a real skill

    It must be connected to an implementation (derived from SkillBase), that will be called when the wrapper is executed
    """

    def __init__(self, ptype, plabel, instanciator=None):
        super(SkillWrapper, self).__init__(NoProcessor())
        # Description
        self._type = ptype
        self._label = plabel
        self._has_instance = False
        self._progress_code = 0
        self._progress_msg = ""
        if instanciator:
            instanciator.assign_description(self)

    # HACK for accessing progress member variable due to shitty implementation -.-
    @property
    def label(self):
        if not self._has_instance:
            return super(SkillWrapper, self).label
        return self._instance.label
    # END of hacky stuff

    def getLightCopy(self):
        """
        Makes a light copy (only description and params)
        """
        p = SkillWrapper(self._type, self._label)
        p._available_for_planning = self._available_for_planning
        p._params = deepcopy(self._params)
        p._remaps = deepcopy(self._remaps)
        p._description = deepcopy(self._description)
        p._pre_conditions = deepcopy(self._pre_conditions)
        p._hold_conditions = deepcopy(self._hold_conditions)
        p._post_conditions = deepcopy(self._post_conditions)
        if self._has_instance:
            p._has_instance = self._has_instance
            p._instance = self._instance
        if self._state != State.Uninitialized:
            p._state = copy(self._state)
            p._wmi = self._wmi
        return p

    def hasInstance(self):
        return self._has_instance

    def setInstance(self, instance):
        """
        instance must be a SkillBase

        All the execution parts get redirected to the instance.

        Instance can change at run-time, but the description will remain fixed
        """
        if self._has_instance:
            self.resetDescription()
        self._description.setDescription(*self.getDescription())
        self._instance = instance
        if isinstance(self._children_processor, NoProcessor) and hasattr(self._instance, '_children_processor'):
            self._children_processor = self._instance._children_processor
        self._has_instance = True
        self._wmi = instance._wmi
        # self._setState(self._instance.getState())
        self._instance.modifyDescription(self)

    def wrapper_expand(self):
        if not self._children or self._instance.expand_on_start:
            self._children = list()
            self._instance.specifyParams(self.getParamsNoRemaps(), False)
            self._instance.expand(self)

    def getInstance(self):
        return self._instance

    def _copyInstanceParams(self):
        self._params.specifyParams(self._instance._params, False)
        for k, p in self._instance._params._params.items():  # Hack to get remapped key back
            if k in self._remaps:
                while self._remaps[k] in self._remaps:  # Hack to work with chained remmapping
                    k = self._remaps[k]
                self._params.specify(self._remaps[k], p.getValues())

    def _mirrorInstance(self):
        self._copyInstanceParams()
        self._progress_msg = self._instance.progress_msg
        self._progress_period = self._instance.progress_period
        self._progress_time = self._instance.progress_time
        self._progress_code = self._instance.progress_code

    def onReset(self):
        return self._instance.reset()

    def onPreempt(self):
        self._instance.specifyParams(self.getParamsNoRemaps(), False)
        res = self._instance.preempt()
        self._mirrorInstance()
        if res is not None:
            return res
        else:
            return self._state

    def onStart(self):
        state = self._instance.start(self.getParamsNoRemaps())
        self._mirrorInstance()
        return state == State.Running

    def execute(self):
        #print '{}:{}'.format(self._label, self._instance)
        self._instance.specifyParams(self.getParamsNoRemaps(), False)
        res = self._instance.tick()
        self._mirrorInstance()
        if res is not None:
            return res
        else:
            self._instance._setState(self._state)
            return self._state

class SkillBase(SkillInterface, object):
    """
    @brief Base class for user's skills
    """
    def _parse_type(self, ptype):
        if ptype.find("skiros:") >= 0:
            return ptype
        elif ptype.find(":") >= 0:
            return "skiros" + ptype
        else:
            return "skiros:" + ptype

    def getLightCopy(self):
        """
        @brief Makes a light copy (only description, params and state)
        """
        p = super(SkillBase, self).getLightCopy()
        if self._state != State.Uninitialized:
            p._instanciator = self._instanciator
        return p

    """ROS2 Node Instance"""
    @property
    def node(self):
        return self._node

    def init(self, wmi, instanciator):
        self._wmi = wmi
        self._instanciator = instanciator
        self._node = instanciator._node # type: Node
        self.createDescription()
        self.generateDefParams()
        self._children_processor = Sequential()
        self._setState(State.Idle)
        self.generateDefConditions()
        self.modifyDescription(self)

    def createDescription(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def expand(self, skill):
        """
        @brief Expand the subtree.
        """
        raise NotImplementedError("Not implemented in abstract class")

    def skill(self, stype, slabel="", remap={}, specify={}, preconditions=[]):
        """
        @brief Utility function to wrapt getSkill and getNode and apply remaps, fixed parameters and pre-conditions
        """
        if isinstance(stype, str):
            s = self.getSkill(stype, slabel)
        else:
            s = self.getNode(stype)
        for k in remap:
            s.remap(k, remap[k])
        for k in specify:
            s.specifyParamDefault(k, specify[k])
        for cond in preconditions:
            s.addPreCondition(cond)
        return s

    def getSkill(self, ptype, plabel):
        """
        @brief Return a skill wrapper initialized
        """
        return SkillWrapper(self._parse_type(ptype), plabel, self._instanciator)

    def getNode(self, children_processor):
        """
        @brief Return a generic node with the required processor type
        """
        return Skill(children_processor.printType(), children_processor, self._wmi)


class Root(SkillInterface):
    """
    @brief Special root node
    """

    def __init__(self, name, wmi=None):
        super(Root, self).__init__()
        self._type = ":Root"
        self._label = name
        self._instance = self  # TODO: fix
        self._children_processor = Sequential()
        if wmi:
            self.init(wmi)

    def createDescription(self):
        return

    def execute(self):
        if self._state == State.Idle:
            return State.Running
        elif self._state == State.Success:
            self._setProgress("End", 1)
        elif self._state == State.Failure:
            self._setProgress("End", -1)
        return self._state


class Skill(SkillInterface):
    """
    @brief Generic skill node
    """

    def __init__(self, name, children_processor=Sequential(), wmi=None):
        super(Skill, self).__init__()
        self._type = ":Skill"
        self._label = name
        self._instance = self  # TODO: fix
        self._children_processor = children_processor
        if wmi:
            self.init(wmi)

    def createDescription(self):
        return

    def execute(self):
        self._setProgress("End", 1)
        return self._state
