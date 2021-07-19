from verification.spec.time import deTimifyFormula
import verification.spec.conf as conf
from verification.choreography.ast_chor import *
from sympy import *
from sympy.logic.boolalg import to_dnf, to_cnf, simplify_logic
from verification.utils.vc import *
from verification.utils.fixed_point import *
from verification.utils.predicate_tracker import *
import copy
import logging
import functools

log = logging.getLogger("CompatibilityChecks")

def choiceAt(node, process_set, choreography):
    # (1) check the predicate: all free var must belong to the same process
    allGuards = { v for gs in node.guarded_states for v in gs.expression.free_symbols }
    choiceCandidates = [ p for p in process_set if any([ v in allGuards for v in p.variables() ]) ]
    if len(choiceCandidates) == 0:
        # (2) next op is a send with the same sender accross all branches
        senders = set()
        for s in node.end_state:
            node2 = choreography.getNode(s)
            if node2.isMessage():
                senders.add(node2.sender)
        if len(senders) == 1:
            sender = senders.pop()
            for p in process_set:
                if p.name() == sender:
                    return p
            raise Exception("choice find: " + sender)
        elif len(senders) > 1:
            raise Exception("choice can be traced back to more than one process: " + str(senders))
        else:
            raise Exception("choice cannot be traced back to any process: " + str(allGuards))
    elif len(choiceCandidates) > 1:
        raise Exception("choice can be traced back to more than one process: " + str(allGuards) + " " + str(choiceCandidates))
    else:
        return choiceCandidates.pop()

def motionForProcess(motions, process):
    candidates = [ m for m in motions if m.id == process.name() ]
    assert(len(candidates) <= 1)
    if len(candidates) == 0:
        return None
    elif len(candidates) == 1:
        return candidates.pop()
    else:
        raise Exception("more than one motion primitive for " + p.name() + " -> " + str(candidates))

class ComputePreds(FixedPointDataflowAnalysis):

    def __init__(self, chor, processes):
        super().__init__(chor, processes, True)

    def initialValue(self, state, node):
        if state == self.chor.start_state:
            return ProcessesPredicatesTracker(self.processes, self.chor.predicate)
        else:
            procNames = self.chor.state_to_processes[state]
            proc = { self.chor.getProcess(name) for name in procNames }
            log.debug("init value for %s is %s", state , proc)
            return ProcessesPredicatesTracker(proc, S.false)

    def motion(self, tracker, motions):
        processes = []
        mps = []
        for motion in motions:
            proc = self.processForMotion(motion)
            mpName = motion.mp_name
            mpArgs = motion.mp_args
            mp = proc.motionPrimitive(mpName, *mpArgs)
            processes.append(proc)
            mps.append(mp)
        #TODO take the thread partition into account for restricting the elements!
        #TODO how to deal with the processes specified in other branches?
        #TODO this assums we have all the processes!
        assert set(processes) == set(tracker._process_set), "processing " + ", ".join(map(str, motions)) + " => " + str(processes) + " != " + str(set(tracker.process_set))
        #relax for the variables that changes
        for mp in mps:
            tracker.relaxVariables(mp.modifies())
        #add the postconditions
        for mp in mps:
            tracker.addFormula(mp.postG())
        return tracker

    def guard(self, tracker, guard):
        tracker.addFormula(guard)
        return tracker


    def _goesTo(self, tracker, pred, succ):
        log.debug("goes from %s to %s with %s", pred, succ, self.chor.state_to_processes[succ])
        assert self.forward
        trackerOld = self.state_to_element[succ]
        res = super()._goesTo(tracker, pred, succ)
        node = self.state_to_node[pred]
        if isinstance(node, Join):
            tracker.join(trackerOld)
        else:
            tracker.restrictTo(self.chor.state_to_processes[succ])
        if log.isEnabledFor(logging.DEBUG):
            log.debug("%s", tracker)
        return res


class CompatibilityCheck:

    def __init__(self, chor, world):
        self.chor = chor
        self.state_to_node = chor.mk_state_to_node()
        self.world = world
        self.processes = world.allProcesses()
        self.state_to_pred = {}
        self.predComputed = False
        self.vcs = []

    def computePreds(self):
        cp = ComputePreds(self.chor, self.processes)
        cp.perform()
        self.state_to_pred = cp.result()
        if log.isEnabledFor(logging.DEBUG):
            log.debug("PREDS")
            for k, v in self.state_to_pred.items():
                log.debug("  %s  -> %s", k, v)
        self.predComputed = True

    def addVC(self, title, formulae, sat = False):
        vc = VC(title, formulae, sat)
        self.vcs.append( vc )

    def getMP(self, node, p):
        motion = motionForProcess(node.motions, p)
        return p.motionPrimitive(motion.mp_name, *motion.mp_args)


    def checkProcessAbstraction(self):
        px, py, pz = symbols('inFpX inFpY inFpZ')
        pointDomain = And(px >= minX, px <= maxX, py >= minY, py <= maxY, pz >= minZ, pz <= maxZ)
        for p in self.processes:
            log.debug("correctness of footprint abstraction for %a", p.name())
            point = p.frame().origin.locate_new("inFp", px * p.frame().i + py * p.frame().j + pz * p.frame().k )
            hypotheses = And(p.invariantG(), pointDomain, p.ownResources(point))
            concl = p.abstractResources(point)
            self.addVC("correctness of footprint abstraction for " + p.name(), [And(hypotheses, Not(concl))])

    def processTopSort(self, processes):
        visited = set()
        stack = []
        def topSort(p):
            visited.add(p)
            for p2 in p.allProcesses():
                if p2 != p and not p2 in visited:
                    topSort(p2)
            stack.insert(0, p)
        for p in processes:
            if not p in visited:
                topSort(p)
        filtered = [ p for p in stack if p in processes]
        return filtered

    # compatibility of motion primitives
    def generateCompatibilityChecks(self):
        assert(self.predComputed)
        obstacles = self.chor.world.obstacles()
        if conf.enableProcessAbstractionCheck:
            self.checkProcessAbstraction()
        for node in self.state_to_node.values():
            log.debug("generateCompatibilityChecks for node %s", node)
            # for the collision check to work, we need to follow the process structure!!
            # otherwise, there are gaps: carrier against arm without knowing where the cart is
            unorderedProcesses = self.chor.getProcessesAt(node)
            processes = self.processTopSort(unorderedProcesses)
            log.debug("unordered processes: %s", unorderedProcesses)
            log.debug("processes: %s", processes)
            if isinstance(node, Motion):
                # state before
                tracker = self.state_to_pred[node.start_state[0]]
                # inv and post extra pred
                tracker2 = tracker.copy()
                for p in processes:
                    mp = self.getMP(node, p)
                    tracker2.relaxVariables(mp.modifies())
                # make the VCs
                overallSpec = self.chor.state_to_contracts[node.start_state[0]] # is FpContract
                composedContracts = None
                # check the composed contract against the node contract
                processInv = And(*[p.invariantG() for p in processes])
                #we need the constraints about the parents for the mounting point
                extra = ExtraInfo(pre = And(tracker.pred()),
                                  always = And(processInv, tracker2.pred()))
                #FIXME Currently some knowledge is in the overallSpec assumptions
                #So the quick hack is to use that rather than write new specs ...
                #We can use that for the collision against obstacles but *not for the refinement of the overallSpec*
                extra2 = extra.strengthen(ExtraInfo(pre = overallSpec.preA(), inv = overallSpec.invA(), post = overallSpec.postA()))
                for p in processes:
                    otherProcesses = [ p2 for p2 in processes if p2 != p ]
                    mp = self.getMP(node, p)
                    log.debug("precondition for process %s", p)
                    self.addVC("precondition of " + mp.name + " for " + p.name() + " @ " + str(node.start_state[0]), [And(extra.pre, extra.always, Not(mp.preG()))])
                    self.vcs.extend(mp.wellFormed(extra))
                    if composedContracts == None:
                        composedContracts = mp
                    else:
                        composedContracts = ComposedContract(composedContracts, mp, dict()) #TODO connection
                    self.vcs.extend(composedContracts.wellFormed(extra))
                    for p2 in obstacles:
                        self.vcs.extend(mp.checkCollisionAgainstObstacle(p2, self.chor.world.frame(), extra = extra2))
                #print("motion: ", str(node))
                #print(extra)
                self.vcs.extend(composedContracts.refines(overallSpec, extra = extra))
            if isinstance(node, Fork):
                log.debug("fork: %s", node)
                overallSpec = self.chor.state_to_contracts[node.start_state[0]] # is FpContract
                composedContracts = None
                extra = ExtraInfo()
                tracker = self.state_to_pred[node.start_state[0]]
                for e1 in node.end_state:
                    e1Spec = self.chor.state_to_contracts[e1] # is FpContract
                    processes = self.chor.getProcessesAt(e1)
                    pInv = And(*[And(p.invariantG(), tracker.pred(p)) for p in processes])
                    extra = ExtraInfo(always = And(extra.always, pInv))
                    if composedContracts == None:
                        composedContracts = e1Spec
                    else:
                        composedContracts = ComposedContract(composedContracts, e1Spec, dict()) #TODO connection
                    self.vcs.extend(composedContracts.wellFormed(extra))
                self.vcs.extend(composedContracts.refines(overallSpec, extra))

    def localChoiceChecks(self):
        for node in self.state_to_node.values():
            if isinstance(node, GuardedChoice):
                choiceAt(node, self.processes, self.chor)

    def generateTotalGuardsChecks(self):
        for node in self.state_to_node.values():
            if isinstance(node, GuardedChoice):
                allGuards = Or(*[ gs.expression for gs in node.guarded_states ])
                self.addVC("total guard @ " + str(node.start_state), [Not(allGuards)])

