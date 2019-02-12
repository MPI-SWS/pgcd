
from sympy import *
from sympy.logic.boolalg import to_dnf, to_cnf, simplify_logic
from utils.vc import *
import copy

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


