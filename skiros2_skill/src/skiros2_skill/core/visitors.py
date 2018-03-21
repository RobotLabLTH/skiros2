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
            vp = VisitorPreempt()
            vp.traverse(root)
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
        self._setState(State.Running)
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

    def processNode(self, procedure):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

    def processChildren(self, procedure):
        """ Use the processor embedded in the procedure """
        return procedure.processChildren(self)

    def postProcessNode(self, procedure):
        """ Not implemented in abstract class. """
        raise NotImplementedError("Not implemented in abstract class")

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
        self._verbose=False
        self._wm = wmi
        self._instanciator = instanciator
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

class VisitorPreempt(VisitorInterface):
    """
    Stops all running nodes
    """
    def __init__(self):
        VisitorInterface.__init__(self)

    def setVerbose(self, verbose):
        self._verbose=verbose

    def processNode(self, procedure):
        if procedure.hasState(State.Running):
            return State.Running
        return State.Success

    def postProcessNode(self, procedure):
        procedure.preempt()
        return State.Success

class VisitorExecutor(VisitorInterface, NodePrinter, NodeExecutor):
    """
    Simulate the procedure execution
    """
    def __init__(self, wmi, instanciator):
        #Execution
        VisitorInterface.__init__(self)
        self._simulate=False
        self._verbose=False
        self._prefix = "->"
        self._indend = 0
        self._wm = wmi
        self._stack = []
        self._tracked_params = []
        self._instanciator = instanciator
        self._params=params.ParamHandler()

    def setVerbose(self, verbose):
        self._verbose=verbose

    def processNode(self, procedure):
        if not procedure.hasState(State.Running):
            state = self.execute(procedure)
        else:
            state = State.Running
        self.indend()
        #print "Pre "+procedure.printState()
        return state

    def postProcessNode(self, procedure):
        state = self.postExecute(procedure)
        self.unindend()
        return state

    def processingDone(self, procedure):
        self.syncParams()
        return True

class VisitorReversibleSimulator(VisitorInterface, NodePrinter, NodeReversibleSimulator):
    """
    Simulate the procedure execution and revert the simulation
    """
    def __init__(self, wmi, instanciator):
        #Execution
        VisitorInterface.__init__(self)
        self._simulate=True
        self._verbose=False
        self._stack = []
        self._tracked_params = []
        self._prefix = "->"
        self._indend = 0
        self._wm = wmi
        self._instanciator = instanciator
        self._params=params.ParamHandler()
        self.forward = NodeMemorizer('Forward')
        self.back = NodeMemorizer('Backward')
        self._execution_branch = []
        self._forget_branch = []
        self.visitor = VisitorPrint(self._wm, self._instanciator)
        self._bound = {}

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
            self.memorize(procedure.id, {"type":procedure.type, "label":procedure.label, "state":procedure.state, "msg":procedure.progress_msg, "code":procedure.progress_code})
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
