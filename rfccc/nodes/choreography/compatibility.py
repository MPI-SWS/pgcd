from spec import *
from ast_chor import *
from sympy import *
from sympy.logic.boolalg import to_dnf, to_cnf, simplify_logic
from utils.vc import *
from utils.fixed_point import *
import copy
import logging
import functools

def choiceAt(node, process_set):
    allGuards = { v for gs in node.guarded_states for v in gs.expression.free_symbols }
    choiceCandidates = [ p for p in process_set if any([ v in allGuards for v in p.variables() ]) ]
    if len(choiceCandidates) == 0:
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

class ProcessPredicatesTracker:

    def __init__(self, process, value = S.true):
        self._process = process
        self._pred = S.false
        for d in getDisjuncts(value):
            accC = S.true
            for c in getConjuncts(d):
                if all([s in process.variables() for s in c.free_symbols]):
                    #print('keep ' + str(c))
                    accC = And(accC, c)
                #else:
                    #print('drop ' + str(c))
            self._pred = Or(self._pred, accC)

    def copy(self):
        return copy.copy(self)

    def pred(self):
        return self._pred

    def addPred(self, pred):
        res = []
        disj1 = getDisjuncts(self._pred)
        for d in disj1:
            eq = []
            nonEq = []
            for c in getConjuncts(d) + [pred]:
                if isinstance(c, Eq) and isinstance(c.args[0], Symbol):
                    eq.append(c)
                else:
                    nonEq.append(c)
            sub = {e.args[0]:e.args[1] for e in eq }
            nonEq = [ e.subs(sub) for e in nonEq ]
            res.append(And(*(eq+nonEq)))
        self._pred = Or(*res)

    def relaxVariables(self, variables):
        #print('relaxVariables ' + str(variables))
        #print('for ' + str(self._pred))
        assert(all(variable in self._process.ownVariables() for variable in variables))
        accD = S.false
        for d in getDisjuncts(self.pred()):
            accC = S.true
            for c in getConjuncts(d):
                if all([not s in variables for s in c.free_symbols]):
                    #print('keep ' + str(c))
                    accC = And(accC, c)
                #else:
                    #print('drop ' + str(c))
            accD = Or(accD, accC)
        self._pred = accD

    def merge(self, tracker):
        assert(self._process == tracker._process)
        self._pred = Or(self._pred, tracker._pred)

    # syntactic, not semantic!
    # contains tracker of all the pred in tracker are in self
    def contains(self, tracker):
        #print(self.pred())
        #print(tracker.pred())
        assert(self._process == tracker._process)
        for dt in getDisjuncts(tracker.pred()):
            notFound = True
            i = 0
            ds = getDisjuncts(self.pred())
            while notFound and i < len(ds):
                if conjContains(ds[i], dt):
                    notFound = False
                i = i+1
            if notFound:
                return False
        return True

    def equals(self, tracker):
        return self.contains(tracker) and tracker.contains(self)

    def __str__(self):
        return str(self._process) + ": " + str(self.pred())


# on predicate tracker for each process
# everything is in place, the idea is to make a copy and apply the operation
class ProcessesPredicatesTracker:

    def __init__(self, process_set, value = S.true):
        self._process_set = process_set
        self._process_to_pred = { p : ProcessPredicatesTracker(p, value) for p in self._process_set }
        self._var_to_process = {}
        for p in self._process_set:
            for v in p.ownVariables():
                assert(not v in self._var_to_process.keys())
                self._var_to_process[v] = p

    def copy(self):
        cpy = copy.copy(self)
        cpy._process_to_pred = {}
        for p in self._process_set:
            cpy._process_to_pred[p] = self._process_to_pred[p].copy()
        return cpy

    def pred(self, process = None):
        if process != None:
            return self._process_to_pred[process].pred()
        else:
            conjs = [self._process_to_pred[p].pred() for p in self._process_set]
            conj = And(*conjs)
            return to_dnf(conj)

    def _findProcess(self, pred):
        ps = set([ self._var_to_process[v] for v in pred.free_symbols ])
        size = len(ps)
        if size == 0:
            return None
        elif size == 1:
            return ps.pop()
        else:
            raise Exception("shared predicate: " + str(ps))

    def addPred(self, pred):
        p = self._findProcess(pred)
        if p == None:
            if pred != S.true:
                logging.warning('ignoring predicate without process "%s": make sure it is not false.', str(pred))
        else:
            self._process_to_pred[p].addPred(pred)

    def addFormula(self, formula):
        assert(len(getDisjuncts(formula)) == 1)
        for c in getConjuncts(formula):
            self.addPred(c)

    def relaxVariables(self, variables):
        for p in self._process_set:
            vs = [ v for v in variables if v in p.ownVariables() ]
            self._process_to_pred[p].relaxVariables(vs)

    def merge(self, tracker):
        for p in self._process_set:
            self._process_to_pred[p].merge(tracker._process_to_pred[p])
    
    def join(self, tracker):
        self.merge(tracker) #TODO fixed once we have the thread partition info!!

    def contains(self, tracker):
        return all([ self._process_to_pred[p].contains(tracker._process_to_pred[p]) for p in self._process_set])

    def equals(self, tracker):
        return all([ self._process_to_pred[p].equals(tracker._process_to_pred[p]) for p in self._process_set])

    def __str__(self):
        acc = "ProcessPredicatesTracker:"
        for p in self._process_set:
            acc = acc + "\n  " + str(self._process_to_pred[p])
        return acc


class ComputePreds(FixedPointDataflowAnalysis):

    def __init__(self, chor, processes, debug = False):
        super().__init__(chor, processes, True, debug)

    def initialValue(self, state, node):
        if state == self.chor.start_state:
            return ProcessesPredicatesTracker(self.processes, self.chor.predicate)
        else:
            return ProcessesPredicatesTracker(self.processes, S.false)

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
        assert set(processes) == set(self.processes), str(processes) + " != " + str(set(self.processes))
        #relax for the variables that changes
        for mp in mps:
            tracker.relaxVariables(mp.modifies())
        #add the postconditions
        for mp in mps:
            tracker.addFormula(mp.post())
        return tracker

    def guard(self, tracker, guard):
        tracker.addFormula(guard)
        return tracker


class CompatibilityCheck:

    def __init__(self, chor, world, minX = -10, maxX = 10, minY = 10, maxY = 10, minZ = 0, maxZ = 2):
        self.chor = chor
        self.state_to_node = chor.mk_state_to_node()
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY
        self.minZ = minZ
        self.maxZ = maxZ
        self.world = world
        self.processes = world.allProcesses()
        self.state_to_pred = {}
        self.predComputed = False
        self.vcs = []

    def computePreds(self, debug = False):
        cp = ComputePreds(self.chor, self.processes, debug = debug)
        cp.perform()
        self.state_to_pred = cp.result()
        self.predComputed = True

    def isTimeInvariant(self, formula):
        #TODO for the moment, what we have is ok but it needs to be checked
        return True

    # compatibility of motion primitives
    def generateCompatibilityChecks(self, debug = False):
        assert(self.predComputed)
        px, py, pz = symbols('inFpX inFpY inFpZ')
        pointDomain = And(px >= self.minX, px <= self.maxX, py >= self.minY, py <= self.maxY, pz >= self.minZ, pz <= self.maxZ)
        for p in self.processes:
            if debug:
                print("correctness of footprint abstraction for " + p.name())
            point = p.frame().origin.locate_new("inFp", px * p.frame().i + py * p.frame().j + pz * p.frame().k )
            hypotheses = And(p.invariant(), pointDomain, p.ownResources(point))
            concl = p.abstractResources(point)
            self.vcs.append( VC("correctness of footprint abstraction for " + p.name(), [And(hypotheses, Not(concl))]) )
        for node in self.state_to_node.values():
            if debug:
                print("generateCompatibilityChecks for node " + str(node))
            if isinstance(node, Motion):
                tracker = self.state_to_pred[node.start_state[0]]
                assumptions = And(*[ p.invariant() for p in self.processes ]) #TODO add the connection as ==
                preState = tracker.pred()
                # a point for the footprint
                point = self.world.frame().origin.locate_new("inFp", px * self.world.frame().i + py * self.world.frame().j + pz * self.world.frame().k )
                # make the VCs
                # precondition
                for p in self.processes:
                    if debug:
                        print("precondition for process " + str(p))
                    motion = motionForProcess(node.motions, p)
                    mp = p.motionPrimitive(motion.mp_name, *motion.mp_args)
                    self.vcs.append( VC("precondition of " + mp.name() + " for " + p.name() + " @ " + str(node.start_state[0]), [And(assumptions, preState, Not(mp.pre()))]) )
                    fs = [And(assumptions, preState, pointDomain, p.abstractResources(point), Not(mp.preFP(point))), And(assumptions, preState, pointDomain, p.ownResources(point), Not(mp.preFP(point)))]
                    self.vcs.append( VC("pre resources of " + mp.name() + " for " + p.name() + " @ " + str(node.start_state[0]), fs) )
                    for p2 in self.processes:
                        if p.name() < p2.name():
                            motion2 = motionForProcess(node.motions, p2)
                            mp2 = p2.motionPrimitive(motion2.mp_name, *motion2.mp_args)
                            fs = [And(assumptions, preState, pointDomain, mp.preFP(point), mp2.preFP(point))]
                            self.vcs.append( VC("no collision in precondition for " + p.name() + " and " + p2.name() + " @ " + str(node.start_state[0]), fs) )
                #frame
                tracker2 = tracker.copy()
                for p in self.processes:
                    motion = motionForProcess(node.motions, p)
                    mp = p.motionPrimitive(motion.mp_name, *motion.mp_args)
                    tracker2.relaxVariables(mp.modifies())
                frame = tracker2.pred()
                #invariant
                inv = frame
                for p in self.processes:
                    if debug:
                        print("invariant (1) for process " + p.name())
                    motion = motionForProcess(node.motions, p)
                    mp = p.motionPrimitive(motion.mp_name, *motion.mp_args)
                    f = mp.inv()
                    assert(self.isTimeInvariant(f))
                    inv = And(inv, deTimifyFormula(p.variables(), f))
                #mp.inv is sat
                self.vcs.append( VC("inv is sat @ " + str(node.start_state[0]), [And(assumptions, inv)], True) )
                #mp.invFP are disjoint
                for p in self.processes:
                    if debug:
                        print("invariant (2) for process " + p.name())
                    motion = motionForProcess(node.motions, p)
                    mp = p.motionPrimitive(motion.mp_name, *motion.mp_args)
                    f1 = mp.invFP(point)
                    assert(self.isTimeInvariant(f1))
                    f1 = deTimifyFormula(p.variables(), f1)
                    for p2 in self.processes:
                        if p.name() < p2.name():
                            motion2 = motionForProcess(node.motions, p2)
                            mp2 = p2.motionPrimitive(motion2.mp_name, *motion2.mp_args)
                            f2 = mp2.invFP(point)
                            assert(self.isTimeInvariant(f2))
                            f2 = deTimifyFormula(p2.variables(), f2)
                            fs = [And(assumptions, inv, pointDomain, f1, f2)]
                            self.vcs.append( VC("no collision in inv for " + p.name() + " and " + p2.name() + " @ " + str(node.start_state[0]), fs) )
                    #process resources are included in invFP
                    fs = [And(assumptions, inv, pointDomain, p.abstractResources(point), Not(f1)), And(assumptions, inv, pointDomain, p.ownResources(point), Not(f1))]
                    self.vcs.append( VC("inv resources of " + mp.name() + " for " + p.name() + " @ " + str(node.start_state[0]), fs) )
                #post:
                post = frame
                for p in self.processes:
                    if debug:
                        print("post (1) for process " + p.name())
                    motion = motionForProcess(node.motions, p)
                    mp = p.motionPrimitive(motion.mp_name, *motion.mp_args)
                    post = And(post, mp.post())
                self.vcs.append( VC("post is sat @ " + str(node.start_state[0]), [And(assumptions, post)], True) )
                #- mp.postFP are disjoint
                for p in self.processes:
                    if debug:
                        print("post (2) for process " + p.name())
                    motion = motionForProcess(node.motions, p)
                    mp = p.motionPrimitive(motion.mp_name, *motion.mp_args)
                    f1 = mp.postFP(point)
                    for p2 in self.processes:
                        if p.name() < p2.name():
                            if debug:
                                print("post (3) for process " + p2.name())
                            motion2 = motionForProcess(node.motions, p2)
                            mp2 = p2.motionPrimitive(motion2.mp_name, *motion2.mp_args)
                            f2 = mp2.postFP(point)
                            fs = [And(assumptions, post, pointDomain, f1, f2)]
                            self.vcs.append( VC("no collision in post for " + p.name() + " and " + p2.name() + " @ " + str(node.start_state[0]), fs) )
                    #process resources are included in postFP
                    fs = [And(assumptions, post, pointDomain, p.abstractResources(point), Not(f1)), And(assumptions, post, pointDomain, p.ownResources(point), Not(f1))]
                    self.vcs.append( VC("post resources of " + mp.name() + " for " + p.name() + " @ " + str(node.start_state[0]), fs) )

    def localChoiceChecks(self):
        for node in self.state_to_node.values():
            if isinstance(node, GuardedChoice):
                choiceAt(node, self.processes)

    def generateTotalGuardsChecks(self):
        for node in self.state_to_node.values():
            if isinstance(node, GuardedChoice):
                allGuards = Or(*[ gs.expression for gs in node.guarded_states ])
                self.vcs.append( VC("total guard @ " + str(node.start_state), [Not(allGuards)]) )

