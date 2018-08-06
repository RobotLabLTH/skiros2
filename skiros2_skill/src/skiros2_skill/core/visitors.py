from skiros2_skill.core.skill_utils import *
from skiros2_common.core.abstract_skill import State, Event

class VisitorInterface:
    """
    Base interface for visitors
    """
    #--------Class functions--------
    def __init__(self):
        #Execution
        self._state=State.Idle
        self._preempt_request = Event()

    def preempt(self):
        self._preempt_request.set()

    def verifyPreempt(self, root):
        if self._preempt_request.is_set():
            self._preempt_request.clear()
            root.visitPreempt(self)
            self._setState(State.Failure)
            return True
        return False

    def _setState(self, state):
        self._state = state

    def getState(self):
        return self._state

    def hasState(self, state):
        return self._state == state

    def traverse(self, root):
        self.processingStart(root)
        if not self.verifyPreempt(root):
            self._setState(root.visit(self))
        self.processingDone(root)
        return self.getState()

    def process(self, procedure):
        #Process node
        state = self.processNode(procedure)
        if state!=State.Running: #Skill start always return Running State until postProcess
            return state
        #Process children
        if procedure.hasChildren():
            state = self.processChildren(procedure)
            if state==State.Running:
                return state
        #Post-process node
        return self.postProcessNode(procedure)

    def processPreempt(self, procedure):
        #Preempt children
        for c in procedure._children:
            c.visitPreempt(self)
        #Preempt node
        procedure.preempt()

    def processNode(self, procedure):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def processChildren(self, procedure):
        """ Use the processor embedded in the procedure """
        return procedure.processChildren(self)

    def postProcessNode(self, procedure):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def processingStart(self, procedure):
        """ Optional - Not implemented in abstract class. """
        return True

    def processingDone(self, procedure):
        """ Optional - Not implemented in abstract class. """
        return True

class VisitorPrint(VisitorInterface, NodePrinter, NodeExecutor):
    """
    Expands and print the whole procedure tree
    """
    def __init__(self, wmi, instanciator):
        #Execution
        VisitorInterface.__init__(self)
        NodePrinter.__init__(self)
        NodeExecutor.__init__(self, wmi, instanciator)
        self._processor = Serial()

    def setVerbose(self, verbose):
        self._verbose=verbose

    def processNode(self, procedure):
        self.init(procedure)
        self.printTree(procedure, self._verbose)
        self.indend()
        return State.Running

    def processChildren(self, procedure):
        """ Use serial processor always """
        return self._processor.processChildren(procedure._children, self)

    def postProcessNode(self, procedure):
        self.unindend()
        return State.Success

class VisitorExecutor(VisitorInterface, NodeExecutor, NodeMemorizer):
    """
    @brief Execute the behavior tree

    Memorizes the nodes with a progress for later process
    """
    def __init__(self, wmi, instanciator):
        #Execution
        VisitorInterface.__init__(self)
        NodeExecutor.__init__(self, wmi, instanciator)

    def setVerbose(self, verbose):
        self._verbose=verbose

    def processNode(self, procedure):
        if not procedure.hasState(State.Running):
            state = self.execute(procedure)
            self.memorizeProgress(procedure)
        else:
            state = State.Running
        return state

    def postProcessNode(self, procedure):
        state = self.postExecute(procedure)
        self.memorizeProgress(procedure)
        return state

    def processPreempt(self, procedure):
        #Preempt children
        for c in procedure._children:
            c.visitPreempt(self)
        #Preempt node
        self.preemptSkill(procedure)
        self.memorizeProgress(procedure)

    def processingStart(self, procedure):
        #self._wm.lock()
        self.reset_memory()
        self.syncParams()
        return True

    def processingDone(self, procedure):
        #self._wm.unlock()
        return True

    def memorizeProgress(self, procedure):
        if procedure.progress_msg:
            self.memorize(procedure.id, {"type":procedure.type,
                                        "label":procedure.label,
                                        "parent":procedure.parent.label if procedure.parent is not None else "",
                                        "state":procedure.state,
                                        "msg":procedure.progress_msg,
                                        "code":procedure.progress_code,
                                        "time": procedure.progress_time})

class VisitorReversibleSimulator(VisitorInterface, NodePrinter, NodeReversibleSimulator):
    """
    Simulate the procedure execution and revert the simulation
    """
    def __init__(self, wmi, instanciator):
        #Execution
        VisitorInterface.__init__(self)
        NodePrinter.__init__(self)
        NodeReversibleSimulator.__init__(self,  wmi, instanciator)

    def setVerbose(self, verbose):
        self._verbose=verbose

    def processNode(self, procedure):
        state = self.execute(procedure)
        self.printTree(procedure)
        self.indend()
        return state

    def postProcessNode(self, procedure):
        state = self.postExecute(procedure)
        self.unindend()
        self.visitor.traverse(self.getExecutionRoot())
        return state

    def processingDone(self, procedure):
        if not self.undoAll():
            return False
        self.back.printMemory()
        return True


class VisitorProgress(VisitorInterface, NodeMemorizer):
    """
    Expands and flattens the whole procedure tree
    """
    def __init__(self):
        #Execution
        VisitorInterface.__init__(self)
        NodeMemorizer.__init__(self, 'VisitorProgress')
        self._verbose = False
        self._processor = Serial()

    def processNode(self, procedure):
        if procedure.progress_msg:
            self.memorize(procedure.id, {"type":procedure.type,
                                        "label":procedure.label,
                                        "state":procedure.state,
                                        "msg":procedure.progress_msg,
                                        "code":procedure.progress_code,
                                        "time": procedure.progress_time})
        return State.Running

    def postProcessNode(self, procedure):
        return State.Success

    def processChildren(self, procedure):
        """ Use serial processor always """
        return self._processor.processChildren(procedure._children, self)

    def processingDone(self, procedure):
        return True

    def getProgress(self):
        return self._tree
