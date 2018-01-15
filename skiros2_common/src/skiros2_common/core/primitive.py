from skiros2_common.core.abstract_skill import SkillCore, State
import skiros2_common.tools.logger as log
from skiros2_common.core.world_element import Element
from datetime import datetime

class PrimitiveBase(SkillCore):
    """
    @brief Base class for primitive skills
    """
    #--------Control functions-------- 
    def tick(self):
        if self.hasState(State.Success) or self.hasState(State.Failure):
            log.error("tick", "Reset required before ticking.")
            return State.Failure
        elif not self.hasState(State.Running):
            log.error("tick", "Start required before ticking.")
            return State.Failure
        else:
            start_time = datetime.now()
            self._setState(self.execute())
            self._updateRoutine(start_time)
            if self._progress_msg!="":
                log.info("[{}]".format(self.printState()), self._progress_msg)
            if self.hasState(State.Success) or self.hasState(State.Failure):
                self.onEnd()
            return self._state
                
    def init(self, wmi, _=None):
        self._wmi = wmi
        self.createDescription()
        self.generateDefParams()
        self.generateDefConditions()
        return self.onInit()
    
    def _updateRoutine(self, time):
        """
        @brief Sync the modified parameters elements with wm
        @time The time to evaluate if a parameter was changed
        """
        for k, p in self.params.iteritems():
            vs = p.values
            if p.dataTypeIs(Element()) and p.hasChanges(time):
                for e in vs:
                    if e._id:
                        self._wmi.updateElement(e)
                    elif self.hasState(State.Success):
                        self._wmi.addElement(e)
        
    #-------- User's functions--------        
    def step(self, msg=""):
        """ 
        @brief Set a running breakpoint 
        """
        self._setProgress(msg)
        return State.Running
        #print '[{}:{}]'.format(self._label, self._progress)
        
    def fail(self, msg, code):
        """ 
        @brief Set a failure state 
        """
        if code > 0:
            code *= -1
        self._setProgress(msg, code)
        return State.Failure

    def success(self, msg=""):
        """ 
        @brief Set a success state 
        """
        self._setProgress(msg)
        return State.Success

    #--------Virtual functions--------
    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True
        
    def onEnd(self):
        """Called just after last execute"""
        pass
