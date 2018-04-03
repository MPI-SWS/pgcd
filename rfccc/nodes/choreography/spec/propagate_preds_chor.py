from spec import *
from ast_chor import *
from sympy import *
from sympy.logic.boolalg import to_dnf, to_cnf, simplify_logic
import copy
import logging
import functools

# need to traverse the choreo, collect the init/pre/post/guard and propagate ...
# For each process we keep a formula in DNF

def getConjuncts(expr):
    return list(And.make_args(expr))

def getDisjuncts(expr):
    return list(Or.make_args(expr))

# conj2 ==> conj1
def conjContains(conj1, conj2):
    c1 = set(getConjuncts(conj1))
    c2 = set(getConjuncts(conj2))
    res = c1 == {S.true} or c1.issubset(c2) or c2 == {S.false}
    #print("contains " + str(c1) + " " + str(c2) + " = " + str(res))
    return res

def choiceAt(node, process_set):
    allGuards = { v for gs in node.guarded_states for v in gs.expression.free_symbols }
    choiceCandidates = [ p for p in process_set if any([ v in allGuards for v in p.variables() ]) ]
    if len(choiceCandidates) == 0:
        raise Exception("choice cannot be traced back to any process: " + str(allGuards))
    elif len(choiceCandidates) > 1:
        raise Exception("choice can be traced back to more than one process: " + str(allGuards) + " " + str(choiceCandidates))
    else:
        return choiceCandidates.pop()

class VC:

    def __init__(self, title, formulas, shouldBeSat = False):
        self.title = title
        self.formulas = [ simplify_logic(f) for f in formulas ] # a list of formula from imprecise (easy to solve) to precise (hard to solve)
        self.sat = shouldBeSat
    
    def __str__(self):
        sat = ""
        if self.sat:
            sat = "sat"
        else:
            sat = "unsat"
        return "VC(" + self.title + ", " + str(self.formulas) + "," + sat + ")"

class ProcessPredicatesTracker:

    def __init__(self, process):
        self._process = process
        self._pred = S.true
    
    def copy(self):
        return copy.copy(self)

    def pred(self):
        return self._pred

    def addPred(self, pred):
        disj1 = getDisjuncts(self._pred)
        disj2 = [And(pred, d) for d in disj1]
        res =  Or(*disj2)
        self._pred = res
    
    def relaxVariables(self, variables):
        #print('relaxVariables ' + str(variables))
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


# on predicate tracker for each process
# everything is in place, the idea is to make a copy and apply the operation
class ProcessesPredicatesTracker:

    def __init__(self, process_set):
        self._process_set = process_set
        self._process_to_pred = { p : ProcessPredicatesTracker(p) for p in self._process_set }
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

    def contains(self, tracker):
        return all([ self._process_to_pred[p].contains(tracker._process_to_pred[p]) for p in self._process_set])
    
    def equals(self, tracker):
        return all([ self._process_to_pred[p].equals(tracker._process_to_pred[p]) for p in self._process_set])


class CompatibilityCheck:

    def __init__(self, state_to_node, start_state, world):
        self.start_state = start_state
        self.state_to_node = state_to_node
        self.world = world
        self.processes = world.allProcesses()
        self.node_to_pred = {}
        for n in state_to_node.values():
            self.node_to_pred[n] = ProcessesPredicatesTracker(self.processes)
        self.predComputed = False
        self.vcs = []

    def _guard(self, pred, guard, succ):
        trackerSrc = self.node_to_pred[pred]
        tracker = trackerSrc.copy()
        tracker.addFormula(guard)
        trackerOld = self.node_to_pred[succ]
        self.node_to_pred[succ] = tracker
        return not trackerOld.equals(tracker)

    def _motion(self, pred, motions, succ):
        #TODO how to deal with the processes specified in other branches?
        #TODO this assums we have all the processes!
        trackerSrc = self.node_to_pred[pred]
        tracker = trackerSrc.copy()
        #relax for the variables that changes
        for mp in motions:
            tracker.relaxVariables(mp.modifies())
        #add the postconditions
        for mp in motions:
            tracker.addFormula(mp.post())
        trackerOld = self.node_to_pred[succ]
        self.node_to_pred[succ] = tracker
        return not trackerOld.equals(tracker)
    
    # for operation that do not change the predicates
    def _transfer(self, pred, succ):
        p = self.node_to_pred[pred]
        s = self.node_to_pred[succ]
        changed = not p.equals(s)
        self.node_to_pred[succ] = p
        return changed

    def _merge(self, preds, succ):
        tracker = ProcessesPredicatesTracker(self.processes)
        for p in preds:
            tracker.merge(self.node_to_pred[p])
        trackerOld = self.node_to_pred[succ]
        self.node_to_pred[succ] = tracker
        return not trackerOld.equals(tracker)

    def motionForProcess(self, motions, process):
        candidates = [ m for m in motions if m.id == process.name() ]
        assert(len(candidates) <= 1)
        if len(candidates) == 0:
            return None
        elif len(candidates) == 1:
            return candidates.pop()
        else:
            raise Exception("more than one motion primitive for " + p.name() + " -> " + str(candidates))
    
    def processForMotion(self, processes, motion):
        candidates = [ p for p in processes if p.name() == motion.id ]
        assert(len(candidates) <= 1)
        if len(candidates) == 0:
            return None
        elif len(candidates) == 1:
            return candidates.pop()
        else:
            raise Exception("more than one process for motion primitive " + motion.id + " -> " + str(candidates))

    # a fixed point to compute the pred at each point in the choreography
    # TODO smarter fixed point with a work queue
    def computePreds(self):
        changed = True
        while changed:
            changed = False
            for node in self.state_to_node.values():
                if isinstance(node, Message):
                    succ = self.state_to_node[node.end_state[0]]
                    res = self._transfer(node, succ)
                    changed = changed or res
                elif isinstance(node, GuardedChoice):
                    for gs in node.guarded_states:
                        guard = gs.expression
                        succ = self.state_to_node[gs.id]
                        res = self._guard(node, guard, succ)
                        changed = changed or res
                elif isinstance(node, Merge):
                    preds = [ self.state_to_node[p] for p in node.start_state ]
                    succ = self.state_to_node[node.end_state[0]]
                    res = self._merge(preds, succ)
                    changed = changed or res
                elif isinstance(node, Fork):
                    for s in node.end_state:
                        succ = self.state_to_node[s]
                        res = self._transfer(node, succ)
                        changed = changed or res
                elif isinstance(node, Join):
                    # TODO probably not the best approach
                    preds = [ self.state_to_node[p] for p in node.start_state ]
                    succ = self.state_to_node[node.end_state[0]]
                    res = self._merge(node, preds, succ)
                    changed = changed or res
                elif isinstance(node, Motion):
                    processes = []
                    mps = []
                    for motion in node.motions:
                        proc = self.processForMotion(self.processes, motion)
                        mpName = motion.mp_name
                        mpArgs = motion.mp_args
                        mp = proc.motionPrimitive(mpName, *mpArgs)
                        processes.append(proc)
                        mps.append(mp)
                    assert(set(processes) == set(self.processes))
                    succ = self.state_to_node[node.end_state[0]]
                    res = self._motion(node, mps, succ)
                    changed = changed or res
                elif isinstance(node, End):
                    pass
        self.predComputed = True

    def isTimeInvariant(self, formula):
        #TODO for the moment, what we have is ok but it needs to be checked
        return True

    # compatibility of motion primitives
    def generateCompatibilityChecks(self):
        assert(self.predComputed)
        for node in state_to_node.values():
            if isinstance(node, Motion):
                tracker = self.state_to_node[node]
                assumptions = And(*[ p.invariant for p in self.processes ]) #TODO add the connection as ==
                preState = tracker.pred()
                # a point for the footprint
                px, py, pz = symbols('inFpX inFpY inFpZ')
                point = self.world.frame().origin.locate_new( px * self.world.frame().i + py * self.world.frame().j + pz * self.world.frame().k )
                # make the VCs
                # precondition
                for p in self.processes:
                    motion = self.motionForProcess(node.motions, p)
                    self.vcs.append( VC("precondition of " + motion.id + " for " + p.name + " @ " + str(node.id), [And(assumptions, preState, Not(motion.pre()))]) )
                    fs = [And(assumptions, preState, p.abstractResources(point), Not(motion.preFP(point))), And(assumptions, preState, p.ownResources(point), Not(motion.preFP(point)))]
                    self.vcs.append( VC("pre resources of " + motion.id + " for " + p.name + " @ " + str(node.id), fs) )
                    for p2 in self.processes:
                        if p.name() < p2.name():
                            motion2 = self.motionForProcess(node.motions, p2)
                            fs = [And(assumptions, preState, motion.preFP(point), motion2.preFP(point))]
                            self.vcs.append( VC("no collision in precondition  for " + p.name + " and " + p2.name() + " @ " + str(node.id), fs) )
                #invariant
                inv = True
                for p in self.processes:
                    motion = self.motionForProcess(node.motions, p)
                    f = motion.inv()
                    assert(self.isTimeInvariant(f))
                    inv = And(inv, deTimifyFormula(p.variables(), f))
                #mp.inv is sat
                self.vcs.append( VC("inv is sat @ " + str(node.id), [And(assumptions, inv)], True) )
                #mp.invFP are disjoint
                for p in self.processes:
                    motion = self.motionForProcess(node.motions, p)
                    f1 = motion.invFP(point)
                    assert(self.isTimeInvariant(f1))
                    f1 = deTimifyFormula(p.variables(), f1)
                    for p2 in self.processes:
                        if p.name() < p2.name():
                            motion2 = self.motionForProcess(node.motions, p2)
                            f2 = motion2.invFP(point)
                            assert(self.isTimeInvariant(f2))
                            f2 = deTimifyFormula(p2.variables(), f2)
                            fs = [And(assumptions, inv, f1, f2)]
                            self.vcs.append( VC("no collision in inv for " + p.name + " and " + p2.name() + " @ " + str(node.id), fs) )
                    #process resources are included in invFP
                    fs = [And(assumptions, inv, p.abstractResources(point), Not(f1)), And(assumptions, inv, p.ownResources(point), Not(f1))]
                    self.vcs.append( VC("inv resources of " + motion.id + " for " + p.name + " @ " + str(node.id), fs) )
                #post:
                post = True
                for p in self.processes:
                    motion = self.motionForProcess(node.motions, p)
                    inv = And(post, motion.post())
                self.vcs.append( VC("post is sat @ " + str(node.id), [And(assumptions, post)], True) )
                #- mp.postFP are disjoint
                for p in self.processes:
                    motion = self.motionForProcess(node.motions, p)
                    f1 = motion.postFP(point)
                    for p2 in self.processes:
                        if p.name() < p2.name():
                            motion2 = self.motionForProcess(node.motions, p2)
                            f2 = motion2.postFP(point)
                            fs = [And(assumptions, post, f1, f2)]
                            self.vcs.append( VC("no collision in post for " + p.name + " and " + p2.name() + " @ " + str(node.id), fs) )
                    #process resources are included in postFP
                    fs = [And(assumptions, post, p.abstractResources(point), Not(f1)), And(assumptions, post, p.ownResources(point), Not(f1))]
                    self.vcs.append( VC("post resources of " + motion.id + " for " + p.name + " @ " + str(node.id), fs) )

    def localChoiceChecks(self):
        for node in self.state_to_node.values():
            if isinstance(node, GuardedChoice):
                choiceAt(node, self.processes)

    def generateTotalGuardsChecks(self):
        for node in self.state_to_node.values():
            if isinstance(node, GuardedChoice):
                allGuards = Or(*[ gs.expression for gs in node.guarded_states ])
                self.vcs.append( VC("total guard @ " + str(node.start_state), [Not(allGuards)]) )

