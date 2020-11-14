from .visitors import *
import rospy
from collections import defaultdict


class VisitorOptimizer(VisitorInterface, NodePrinter, NodeReversibleSimulator):
    """
    Simulate the execution back and fort to find an optimized execution sequence, then revert the simulation.

    The optimized execution sequence get stored in a new behaviour tree, and can be retrieved with 'getExecutionTree'
    """

    def __init__(self, wmi, instanciator):
        # Execution
        self._simulate = True
        self._verbose = False
        self._state = State.Idle
        self._preempt_request = Event()
        self._prefix = "->"
        self._indend = 0
        self._wm = wmi
        self._instanciator = instanciator
        self._stack = []
        self._tracked_params = []
        self._params = params.ParamHandler()
        self._processor = Serial()
        self.static = NodeMemorizer('Static')
        self.forward = NodeMemorizer('Forward')
        self.back = NodeMemorizer('Backward')
        self._execution_branch = []
        self._forget_branch = []
        self._static_branch = []
        self.visitor = VisitorPrint(self._wm, self._instanciator)
        self._bound = {}
        self._time_keep = TimeKeepers()

    def setVerbose(self, verbose):
        self._verbose = verbose

    def checkConflicts(self, p1, p2):
        # Rule 2: can-t go before a procedure with conflicting conditions
        for c1 in p1._post_conditions:
            for c2 in p2._post_conditions:
                if c1.hasConflict(c2):
                    log.warn("RULE2.1", "{}:{} has conflict with {}:{}".format(p1._label, c1.getDescription(), p2._label, c2.getDescription()))
                    return False
        for c1 in p1._post_conditions:
            for c2 in p2._pre_conditions:
                if c1.hasConflict(c2):
                    log.warn("RULE2.2", "{}:{} has conflict with {}:{}".format(p1._label, c1.getDescription(), p2._label, c2.getDescription()))
                    return False
        # Heuristic 1: no need to go before another equal procedure
        if p1._label == p2._label:
            log.warn("STOP", "{}:{} are the same procedure.".format(p1._label, p2._label))
            return False
        return True

    def checkRules(self, p1, p2):
        """
        Return true if the 3 rules are respected:
            Rule 1: preconditions have to hold
            Rule 2: can-t go before a procedure with inverse postconditions
            Rule 3: parent preconditions involving params of child have to hold
        """
        # Rule 1: preconditions have to hold
        if p1.checkPreCond(self._verbose):
            log.warn("RULE1", "{} preCondCheck failed.".format(p1._label))
            p1.checkPreCond(True)
            return False

        if not self.checkConflicts(p1, p2):
            return False

        #log.warn("CheckingConds", "{}1:{} \n {}2:{}".format(p1._label, self.printParams(p1._params), p2._label, self.printParams(p2._params)))
        # Rule 3: node moving out of its parent: preconditions involving params of child have to hold as well
        if p1.inSubtreeOf(p2):
            s = p1._label + ' is child of ' + p2._label + '. Adding conditions: '
            for c2 in p2._pre_conditions:
                if [key for key, _ in p1._params.getParamMap().items() if key in c2.getKeys()]:
                    dont_add = False
                    for c1 in p1._pre_conditions:
                        if c1.isEqual(c2):
                            dont_add = True
                    if dont_add:
                        continue
                    s += c2.getDescription()
                    s += ", "
                    p1.addPreCondition(c2)
                    for key in c2.getKeys():
                        if not p1._params.hasParam(key):
                            p1._params.getParamMap()[key] = p2._params.getParamMap()[key]
            log.info("RULE3", s)
        return True

    def checkRule5(self, p):
        """
        Rule 5: node moving into a node with parallel processor: maximum 1 conflict with other childs
        """
        if self.back.recall()[1] == "postExecute" and isinstance(self.back.recall()[0]._children_processor, Parallel):
            parent = self.back.recall()[0]
            if self.forward.recall()[0] != parent:
                log.info("checkRule5", "Entering parallel node {}.".format(parent._label))
                conflicts_count = 0
                while self.undo():
                    p.setInput(self._params)
                    if not self.checkRules(p, self.back.recall()[0]):
                        conflicts_count += 1
                        self.redo()
                        self.makeStaticPrevious()
                        #check_point = self.forward.recall()
                        i = -1
                        while self.forward.recall(i)[0] != parent and self.forward.hasIndex(i):
                            p2 = self.forward.recall(i)[0]
                            if not self.checkConflicts(p, p2):
                                conflicts_count += 1
                            i -= 1
                        break
                    if self.forward.recall()[0] == parent:  # Go back until exit from parallel node
                        break
                # while self.forward.recall()[0]!= check_point[0] and self.forward.recall()[1]!=check_point[1]:
                #    self.redo()
                self.makeStaticAll(False)
                if conflicts_count == 0:
                    log.warn("checkRule5", "{} {} no conflicts.".format(parent._label, p._label))
                    while self.back.recall()[0] != parent and self.back.recall()[1] == "execute":
                        self.undo()
                    return True
                elif conflicts_count == 1:
                    log.warn("checkRule5", "{} {} has 1 conflict.".format(parent._label, p._label))
                    return False
                else:
                    log.warn("checkRule5", "{} {} has {} conflicts.".format(parent._label, p._label, conflicts_count))
                    while self.back.recall()[0] != parent:
                        self.redo()
                    return False
        return True

    def processNode(self, procedure):
        # initialize the procedure
        log.warn("PROCESSING", procedure._label)
        start_time = rospy.Time.now()
        if not self.initAndParametrize(procedure):
            log.error(procedure._label, 'Initialization failed')
            print(self.printParams(procedure._params))
            return False
        # discard redundant
        if not procedure.checkPostCond() and procedure.hasPostCond():
            log.info('discarded', 'Redundand procedure {}'.format(procedure._label))
            return True
        processor = None
        # revert until possible
        if self._verbose:
            log.info("Init done ({}). Start undo. ".format((rospy.Time.now() - start_time).to_sec()))
            start_time = rospy.Time.now()

        if self.forward.hasMemory():
            while self.undo():
                procedure.setInput(self._params)
                if not self.checkRules(procedure, self.back.recall()[0]):
                    self.redo()
                    processor = Serial
                    break
                if not self.checkRule5(procedure):
                    processor = Serial
                    break

        # Insert
        if self._verbose:
            log.info("Undo done ({}). Start swap. ".format((rospy.Time.now() - start_time).to_sec()))
            start_time = rospy.Time.now()
        while True:
            if not self.back.recall():
                break
            if not procedure.inSubtreeOf(self.back.recall()[0]) and self.back.recall()[1] != 'postExecute':
                break
            if self._verbose:
                log.info("SWAP", self.back.recall()[0]._label)
            if not self.redo():
                log.error('Redo failed')
                print(self.printParams(procedure._params))
                return False
            processor = None

        if self._verbose:
            log.info("Swap done ({}). Insert. ".format((rospy.Time.now() - start_time).to_sec()))
            start_time = rospy.Time.now()
        if not self.execute(procedure, processor=processor):
            log.error('Execute failed')
            print(self.printParams(procedure._params))
            return False
        if self._verbose:
            log.info("Insert done. Insert parallel. ")

        if False and self.back.hasMemory() and len(procedure._children) > 0:  # Issue 3: node with childs
            if not self.checkRule4(procedure):
                raise
            log.error("UNDO", procedure._label)
            while procedure != self.forward.recall()[0]:
                self.undo()
            self.undo()
            while procedure != self.forward.recall()[0]:
                self.undo()
            log.error("DONE")
        if procedure.hasChildren():
            self.redoAll()
        self.indend()
        return True

    def checkRule4(self, procedure):
        """
        Return true if the rule 4 is respected:
            Rule 4: procedure doesn-t interfere with subsequent procedure's preconditions
        """
        if not self.postExecute(procedure):
            raise
        if not self.back.hasMemory():
            return True
        if not self.redo(Parallel):  # Issue 2: parallel and serial of all nexts...
            raise
            self.erase()
            return False
        return True

    def processChildren(self, procedure):
        """ Bypass procedure processor, and use serial processor always """
        return self._processor.processChildren(procedure._children, self)

    def postProcessNode(self, procedure):
        if not procedure.checkPostCond() and procedure.hasPostCond():
            log.info('discarded', 'Redundand procedure {}'.format(procedure._label))
            return True
        while not self.checkRule4(procedure):
            self.redo()
        if self._verbose:
            log.info("Insert procedure done. Redo all. ".format())
            start_time = rospy.Time.now()
        self.redoAll()
        self.unindend()
        if self._verbose:
            log.info("Redoall done ({}).".format((rospy.Time.now() - start_time).to_sec()))
        # self.visitor.traverse(self.getExecutionRoot())
        return True

    def processingDone(self, procedure):
        self.freezeExecutionTree()
        if not self.undoAll():
            return False
        return True
