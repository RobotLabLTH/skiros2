from skiros2_common.core.abstract_skill import State

"""
Collection of processor, defining how a skill visits its children
"""
class Serial():
    """
    @brief Process children serially. Also succeded one are re-executed
    """
    def printType(self):
        return '->'

    def processChildren(self, children, visitor):
        """
        Serial processor - return on first fail, or return success
        """
        for c in children:
            state = c.visit(visitor)
            if state!=State.Success:
                return state
        return State.Success
        
class Sequential():
    """
    @brief Process children sequentially. Succeded ones are skipped
    """
    def printType(self):
        return '->'

    def processChildren(self, children, visitor):
        """
        Serial processor - return on first fail, or return success
        """
        for c in children:
            if c.state!=State.Success:
                state = c.visit(visitor)
                if state!=State.Success:
                    return state
        return State.Success

class Selector():
    """
    Process children sequentially.
    """
    def printType(self):
        return '?'

    def processChildren(self, children, visitor):
        """
        Serial processor - return on first running/success, or return failure
        """
        for c in children:
            state = c.visit(visitor)
            if state==State.Success or state==State.Running:
                if state==State.Success:
                    self.stopAll(children, visitor)
                return state
        return State.Failure

    def stopAll(self, children, visitor):
        for c in children:
            if c._state==State.Running:
                c.preempt()

class ParallelFf():
    """
    Parallel First Fail - Process children in parallel. Stop all processes if a child fails.
    """
    def printType(self):
        return '|ff|'

    def processChildren(self, children, visitor):
        state = State.Success
        for c in children:
            cstate = c.visit(visitor)
            if cstate==State.Running:
                state = State.Running
            if cstate==State.Failure:
                self.stopAll(children, visitor)
                return State.Failure
        return state

    def stopAll(self, children, visitor):
        for c in children:
            if c._state==State.Running:
                c.preempt() #TODO: the visitor should be included in the loop

class ParallelFs():
    """
    Parallel First Stop - Process children in parallel. Stop all processes if a child ends (success or fail).
    """
    def printType(self):
        return '|fs|'

    def processChildren(self, children, visitor):
        for c in children:
            state = c.visit(visitor)
            if state!=State.Running:
                self.stopAll(children, visitor)
                return state
        return State.Running

    def stopAll(self, children, visitor):
        for c in children:
            if c._state==State.Running:
                c.preempt()
#Decorators
class Loop():
    def __init__(self, processor):
        self._processor = processor

    def printType(self):
        return 'Loop({})'.format(self._processor.printType())

    def processChildren(self, children, visitor):
        """
        Repeat execution
        """
        state = self._processor.processChildren(children, visitor)
        if state==State.Failure:
            return state
        return State.Running

class NoFail():
    def __init__(self, processor):
        self._processor = processor

    def printType(self):
        return 'NoFail({})'.format(self._processor.printType())

    def processChildren(self, children, visitor):
        """
        Ignore failed execution
        """
        self._processor.processChildren(children, visitor)
        return True
