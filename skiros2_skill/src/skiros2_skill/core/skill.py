import skiros2_common.core.params as params
import skiros2_common.tools.logger as log
from processors import *
from copy import deepcopy
from copy import copy
from collections import defaultdict
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
        #Params
        self._remaps={}
        #Connections
        self._parent = None
        self._children=[]
        self._children_processor = children_processor
        #Caches
        self._was_simulated = False
        self._max_cache = 2
        self._params_cache = []
        self._input_cache = []
        self._remaps_cache = defaultdict(list)

    def getLightCopy(self):
        """
        Makes a light copy (only description, params and state)
        """
        p = self.__class__(self._children_processor)
        p._children_processor = deepcopy(self._children_processor)
        p._type=copy(self._type)
        p._label=copy(self._label)
        p._params = deepcopy(self._params)
        p._remaps = deepcopy(self._remaps)
        p._description = deepcopy(self._description)
        p._pre_conditions = deepcopy(self._pre_conditions)
        p._hold_conditions = deepcopy(self._hold_conditions)
        p._post_conditions = deepcopy(self._post_conditions)
        if self._state!=State.Uninitialized:
            p._state = copy(self._state)
            p._wmi = self._wmi
        return p

    def _setState(self, state):
        self._state = state
        self._state_change.set()

    def _clearRemaps(self):
        """
        Clear remaps
        """
        for r1, r2 in reversed(self._remaps.iteritems()):
            self.remap(r2, r1)
        self._remaps={}
        self._remaps_cache={}

    def _copyRemaps(self, skill):
        """
        Copy the remaps of another skill. Called automatically when the skill is added as a child
        """
        for r1, r2 in skill._remaps.iteritems():
            self.remap(r1, r2)

    def _revertRemaps(self):
        """
        Revert remaps. Just used in revertInput
        """
        remapid = len(self._params_cache)
        try:
            if self._remaps_cache[remapid]:
                for remap in self._remaps_cache[remapid]:
                    log.warn("REMAP", "Revert {} {}".format(remap[0], remap[1]))
                    print self._remaps_cache
                    self._remaps.pop(remap[0])
                    self.remap(remap[1], remap[0], False)
                del self._remaps_cache[remapid]
        except:
            print self.printInfo(True)
            print self._remaps_cache
            raise

    def getParamsNoRemaps(self):
        ph = params.ParamHandler()
        ph.reset(self._params.getCopy())
        for r1, r2 in self._remaps.iteritems():
            ph.remap(r2, r1)
        return ph

    def remap(self, initial_key, target_key, record=True):
        """
        Remap a parameter to a new key
        """
        #log.error(self._label, "remap {} {} {}".format(self, initial_key, target_key))
        if self._remaps.has_key(initial_key):
            if self._remaps[initial_key]==target_key:#Redundant
                #log.warn(self._label, "Ignoring redundant remap {}->{}".format(initial_key, target_key))
                return
            else:
                #log.warn(self._label, "Key {} already remapped to {}. Can t remap to {}".format(initial_key, self._remaps[initial_key], target_key))
                return
        if self._remaps.has_key(target_key):
            #log.warn(self._label, "Ignoring circular remap {}->{}".format(initial_key, target_key))
            return

        if self._params.hasParam(target_key):
            log.error(self._label, "Key {} already present in the map, remapping can shadow a parameter.".format(target_key))
            return
        for c in self._children:
            c.remap(initial_key, target_key)
        #Remaps
        self._params.remap(initial_key, target_key)
        for c in self._pre_conditions:
            c.remap(initial_key, target_key)
        for c in self._hold_conditions:
            c.remap(initial_key, target_key)
        for c in self._post_conditions:
            c.remap(initial_key, target_key)
        #Records
        if record:
            self._remaps[initial_key] = target_key
            remapid = len(self._params_cache)
            if remapid>0: #If I have params cached, I cache also the remap to revert it afterwards
                self._remaps_cache[remapid].append((initial_key, target_key))

    def init(self, wmi):
        self._wmi = wmi
        self.createDescription()
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

    def printInfo(self, verbose=False):
        s = "{}-{} ".format(self._type,self._label)
        if verbose:
            s += "["
            s += self._params.printState() + ']\n'
            s += self.printConditions()
            s += "Remaps: \n"
            for r1, r2 in self._remaps.iteritems():
                s += r1+"->"+r2
        else:
            s += "\n"
        return s

    def printState(self, verbose=False):
        s = "{}-{}({})".format(self._type, self._label, self._state)
        if verbose:
            if self.hasInstance():
                if self._children:
                    s += "({})".format(self._children_processor.printType())
                #s += "({})".format(self._state)
                s += "[{}]".format(self._params.printState())
                #s += "\n[Remaps: {}]".format(self._remaps)
                #s += "[Remaps cache: {}]".format(self._remaps_cache)
                #s += "[{}]".format(self.getModifiedParams())
            else:
                s += "({})".format('abstract')
        return s

    def getParent(self):
        return self._parent

    def hasChildren(self):
        return len(self._children)>0

    def addChild(self, p, latch=False):
        if isinstance(p, list):
            for i in p:
                i._parent = self
                self._children.append(i)
                i._copyRemaps(self)
        else:
            p._parent = self
            self._children.append(p)
            p._copyRemaps(self)
        if latch and len(self._children)>1:
            for c in self._children[-2]._post_conditions:
                for key in c.getKeys():
                    if not p._params.hasParam(key):
                        p._params._params[key] = deepcopy(self._children[-2]._params._params[key])
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
        #Erase the oldest cache if the max lenght is reached
        if len(self._params_cache)>self._max_cache:
            self._popCache()

    def _popCache(self):
        self._params_cache.pop(0)
        for i in range(self._max_cache+1):
            if not self._remaps_cache.has_key(i):
                continue
            del self._remaps_cache[i]
            if self._remaps_cache.has_key(i+1):
                self._remaps_cache[i] = self._remaps_cache[i+1]

    def revertInput(self):
        if not self._params_cache:
            log.warn("revertInput", "No cache available, can't revert input.")
            return None
        self._revertRemaps()
        self._params.reset(deepcopy(self._params_cache.pop()))
        return deepcopy(self._input_cache.pop())

    def tick(self):
        if not self.hasState(State.Running):
            raise Exception("Node must be started before ticking. Curr state: {}".format(self._state))
        self._setState(self.execute())
        #TODO: Removing this has consequences... I will see them in the optimization
        #if input_params != None:
        #    input_params.specifyParams(self._params, False)
        return self._state

    #--------User functions--------
    def setChildrenProcessor(self, processor):
        self._children_processor = processor

    def resetDescription(self, other=None):
        if other:
            self._params.reset(self._description._params.merge(other._params))
        else:
            self._params = deepcopy(self._description._params)
        self._pre_conditions = deepcopy(self._description._pre_conditions)
        self._hold_conditions = deepcopy(self._description._hold_conditions)
        self._post_conditions = deepcopy(self._description._post_conditions)
        self._children = []

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

    def addPreCondition(self, condition):
        self._pre_conditions.append(condition)
        for r1, r2 in self._remaps.iteritems():
            self._pre_conditions[-1].remap(r1, r2)

    def addHoldCondition(self, condition):
        self._hold_conditions.append(condition)
        for r1, r2 in self._remaps.iteritems():
            self._hold_conditions[-1].remap(r1, r2)

    def addPostCondition(self, condition):
        self._post_conditions.append(condition)
        for r1, r2 in self._remaps.iteritems():
            self._post_conditions[-1].remap(r1, r2)

    def processChildren(self, visitor):
        """
        """
        return self._children_processor.processChildren(self._children, visitor)

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
        return State.Success

class SkillWrapper(SkillInterface):
    """
    A skill wrapper is a placeholder for a real skill

    It must be connected to an implementation (derived from SkillBase), that will be called when the wrapper is executed
    """
    def __init__(self, ptype, plabel, instanciator=None):
        super(SkillWrapper, self).__init__()
        #Description
        self._type=ptype
        self._label=plabel
        self._has_instance=False
        if instanciator:
            instanciator.assignDescription(self)


    ## HACK for accessing progress member variable due to shitty implementation -.-

    @property
    def id(self):
        if not self._has_instance: return super(SkillWrapper, self).id
        return self._instance.id

    @property
    def progress_code(self):
        if not self._has_instance: return super(SkillWrapper, self).progress_code
        return self._instance.progress_code

    @property
    def progress_msg(self):
        if not self._has_instance: return super(SkillWrapper, self).progress_msg
        return self._instance.progress_msg

    @property
    def label(self):
        if not self._has_instance: return super(SkillWrapper, self).label
        return self._instance.label

    @property
    def type(self):
        if not self._has_instance: return super(SkillWrapper, self).type
        return self._instance.type

    @property
    def state(self):
        if not self._has_instance: return super(SkillWrapper, self).state
        return self._instance.state
    ### END of hacky stuff


    def getLightCopy(self):
        """
        Makes a light copy (only description and params)
        """
        p = SkillWrapper(self._type, self._label)
        p._params = deepcopy(self._params)
        p._remaps = deepcopy(self._remaps)
        p._description = deepcopy(self._description)
        p._pre_conditions = deepcopy(self._pre_conditions)
        p._hold_conditions = deepcopy(self._hold_conditions)
        p._post_conditions = deepcopy(self._post_conditions)
        if self._has_instance:
            p._has_instance=self._has_instance
            p._instance=self._instance
        if self._state!=State.Uninitialized:
            p._state = copy(self._state)
            p._wmi = self._wmi
        return p

    def hasState(self, state):
        #TODO: fix
#        if self._has_instance:
#            if self.getState() != self._instance.getState():
#                print "{}: {} to {}".format(self._label, self.getState(), self._instance.getState())
#                self._setState(self._instance.getState())
        return super(SkillWrapper, self).hasState(state)

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
        self._instance = instance
        self._has_instance = True
        self._wmi = instance._wmi
        instance.expand(self)

    def getInstance(self):
        return self._instance

    def _copyInstanceParams(self):
        self._params.specifyParams(self._instance._params, False)
        for k, p in self._instance._params._params.iteritems():#Hack to get remapped key back
            if k in self._remaps:
                self._params.specify(self._remaps[k], p.getValues())

    def onReset(self):
        return self._instance.reset()

    def onPreempt(self):
        return self._instance.preempt()

    def onStart(self):
        return self._instance.start(self.getParamsNoRemaps())

    def execute(self):
        #print '{}:{}'.format(self._label, self._instance)
        self._instance.specifyParams(self.getParamsNoRemaps(), False)
        self._setState(self._instance.tick())
        self._copyInstanceParams()
        return self._state

class SkillBase(SkillInterface, object):
    """
    Base class for user's skills
    """
    def getLightCopy(self):
        """
        Makes a light copy (only description, params and state)
        """
        p = super(SkillBase, self).getLightCopy()
        if self._state!=State.Uninitialized:
            p._instanciator = self._instanciator
        return p

    def init(self, wmi, instanciator):
        self._wmi = wmi
        self._instanciator = instanciator
        self.createDescription()
        self.generateDefParams()
        self._children_processor = Sequential()
        self._setState(State.Idle)
        self.generateDefConditions()

    def createDescription(self):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def expand(self, skill):
        """
        @brief Expand the subtree.
        """
        raise NotImplementedError("Not implemented in abstract class")

    def getSkill(self, ptype, plabel):
        """
        @brief Return a skill wrapper initialized
        """
        return SkillWrapper(ptype, plabel, self._instanciator)

    def getNode(self, children_processor):
        """
        @brief Return a generic node with the required processor type
        """
        return Skill(children_processor.printType(), children_processor, self._wmi)

class Root(SkillInterface):
    """
    Special root node
    """
    def __init__(self, name, wmi=None):
        super(Root, self).__init__()
        self._type=":Root"
        self._label=name
        self._instance = self#TODO: fix
        self._children_processor = Sequential()
        if wmi:
            self.init(wmi)

    def createDescription(self):
        return

class Skill(SkillInterface):
    """
    Generic skill node
    """
    def __init__(self, name, children_processor=Sequential(),wmi=None):
        super(Skill, self).__init__()
        self._type=":Skill"
        self._label=name
        self._instance = self#TODO: fix
        self._children_processor = children_processor
        if wmi:
            self.init(wmi)

    def createDescription(self):
        return
