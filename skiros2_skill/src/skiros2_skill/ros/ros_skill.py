from skiros2_skill.core.skill import SkillBase, State
from skiros2_resource.ros.resource_layer_interface import ResourceLayerInterface
import skiros2_common.tools.logger as log
from skiros2_common.core.world_element import Element

    
class RosSkill(SkillBase):  
    """
    A skill that executes a primitive on a resource manager
    """    
    def setRosInterface(self, ri, mgr_name):
        self._ri = ri 
        self._mgr_name = mgr_name
        
    def _updateRoutine(self, result, state):
        """
        Add elements to wm
        """
        #log.warn(skill._label, "Sync starts")
        for k, p in result.iteritems():
            vs = p.getValues()
            if p.dataTypeIs(Element()):
                for e in vs:
                    with self._wmi:
                        if e._id:
                            self._wmi.updateElement(e)
                        elif state == State.Success:
                            self._wmi.addElement2(e)
            self._params.specify(k, vs)
                          
    def createDescription(self):
        return
                  
    def expand(self, skill):
        return
        
    def onPreempt(self):
        res = self._ri.preempt(self._mgr_name, self._label)
        if not res:
            log.error("[{}]".format(self._label), "Failed to preempt.")
            return State.Failure
        return res
        
    def onReset(self):
        res = self._ri.reset(self._mgr_name, self._label)
        if not res:
            log.error("[{}]".format(self._label), "Failed to reset.")
            return State.Failure
        return res
        
    def onStart(self):
        res = self._ri.start(self._mgr_name, self._label, self._params.getParamMap())
        if not res:
            log.error("[{}]".format(self._label), "Failed to execute.")
            return State.Failure
        return State.Running
        
    def execute(self):
        state = self._ri.tick(self._mgr_name, self._label, self._params.getParamMap())
        if not state:
            log.error("[{}]".format(self._label), "Failed to execute.")
            return State.Failure
        result = self._ri.getResult(self._mgr_name, self._label)
        self._updateRoutine(result, state)
        return state