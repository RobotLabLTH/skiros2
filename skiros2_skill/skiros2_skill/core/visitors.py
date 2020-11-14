from skiros2_skill.core.skill_utils import *
from skiros2_common.core.abstract_skill import State, Event
import skiros2_common.tools.logger as log
import traceback

class VisitorInterface(object):
    """
    @brief Base interface for visitors
    """
    #--------Class functions--------

    def __init__(self):
        # Execution
        self._state = State.Idle
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
        # Process node
        state = self.processNode(procedure)
        if state != State.Running:  # Skill start always return Running State until postProcess
            return state
        # Process children
        if procedure.hasChildren():
            state = self.processChildren(procedure)
            if state == State.Running:
                return state
        # Post-process node
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

    def processingStart(self, procedure):
        """ Optional - Not implemented in abstract class. """
        return True

    def processingDone(self, procedure):
        """ Optional - Not implemented in abstract class. """
        return True


class VisitorPrint(VisitorInterface, NodePrinter, NodeExecutor, NodeMemorizer):
    """
    Expands and memorize the whole procedure tree. printout if verbose
    """

    def __init__(self, wmi, instanciator, verbose=False):
        # Execution
        VisitorInterface.__init__(self)
        NodePrinter.__init__(self)
        NodeExecutor.__init__(self, wmi, instanciator)
        NodeMemorizer.__init__(self, 'trace')
        self._verbose = verbose
        self._processor = Serial()

    def setVerbose(self, verbose):
        self._verbose = verbose

    def processNode(self, procedure):
        #self.init(procedure)
        #procedure.wrapper_expand()
        if self._verbose:
            self.printTree(procedure, self._verbose)
        self.indend()
        self.memorizeProcedure(procedure)
        return State.Running

    def processChildren(self, procedure):
        """ Use serial processor always """
        return self._processor.processChildren(procedure._children, self)

    def postProcessNode(self, procedure):
        self.unindend()
        return State.Success

    def memorizeProcedure(self, procedure):
        self.memorize(procedure.id, {"type": procedure.type,
                                     "label": procedure.label,
                                     "params": procedure.params,
                                     "processor": procedure._children_processor.printType(),
                                     "parent_id": procedure.parent.id if procedure.parent is not None else -1,
                                     "parent_label": procedure.parent.label if procedure.parent is not None else "",
                                     "state": procedure.state,
                                     "msg": procedure.progress_msg,
                                     "code": procedure.progress_code,
                                     "period": 0.0,
                                     "time": procedure.progress_time})


class VisitorExecutor(VisitorInterface, NodeExecutor, NodeMemorizer):
    """
    @brief Execute the behavior tree

    Memorizes the nodes with a progress for later process
    """

    def __init__(self, wmi, instanciator):
        # Execution
        VisitorInterface.__init__(self)
        NodeExecutor.__init__(self, wmi, instanciator)
        NodeMemorizer.__init__(self, 'trace')

    def setVerbose(self, verbose):
        self._verbose = verbose

    def processNode(self, procedure):
        if not procedure.hasState(State.Running):
            try:
                state = self.execute(procedure)
            except Exception:
                log.error(self.__class__.__name__, traceback.format_exc())
                procedure._instance.onEnd()
                procedure._setProgress("Error on start: {}. Blackboard data: {}".format(traceback.format_exc(), self._params.printState()), -404)
                procedure._setState(State.Failure)
                state = State.Failure
            self.memorizeProgress(procedure)
        else:
            state = State.Running
        return state

    def postProcessNode(self, procedure):
        try:
            state = self.postExecute(procedure)
        except Exception:
            log.error(self.__class__.__name__, traceback.format_exc())
            procedure._instance.onEnd()
            procedure._setProgress("Error on execution: {}. Blackboard data: {}".format(traceback.format_exc(), self._params.printState()), -405)
            procedure._setState(State.Failure)
            state = State.Failure
        self.memorizeProgress(procedure)
        return state

    def processChildren(self, procedure):
        """
        @brief Check hold conditions before ticking childrens
        """
        state = self.checkHold(procedure)
        return state if state!=State.Running else super(VisitorExecutor, self).processChildren(procedure)

    def processPreempt(self, procedure):
        try:
            super(VisitorExecutor, self).processPreempt(procedure)
        except Exception:
            log.error(self.__class__.__name__, traceback.format_exc())
            procedure._instance.onEnd()
            procedure._setProgress("Error on preemption: {}. Blackboard data: {}".format(traceback.format_exc(), self._params.printState()), -406)
            procedure._setState(State.Failure)
        self.memorizeProgress(procedure)

    def processingStart(self, procedure):
        self.reset_memory()
        return True

    def processingDone(self, procedure):
        return True

    def memorizeProgress(self, procedure):
        self.memorize(procedure.id, {"type": procedure.type,
                                     "label": procedure.label,
                                     "params": procedure.params,
                                     "processor": procedure._children_processor.printType(),
                                     "parent_id": procedure.parent.id if procedure.parent is not None else -1,
                                     "parent_label": procedure.parent.label if procedure.parent is not None else "",
                                     "state": procedure.state,
                                     "msg": procedure.progress_msg,
                                     "code": procedure.progress_code,
                                     "period": procedure.progress_period if procedure.progress_period is not None else 0.0,
                                     "time": procedure.progress_time})


class VisitorReversibleSimulator(VisitorInterface, NodePrinter, NodeReversibleSimulator):
    """
    Simulate the procedure execution and revert the simulation
    """

    def __init__(self, wmi, instanciator):
        # Execution
        VisitorInterface.__init__(self)
        NodePrinter.__init__(self)
        NodeReversibleSimulator.__init__(self, wmi, instanciator)

    def setVerbose(self, verbose):
        self._verbose = verbose

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
