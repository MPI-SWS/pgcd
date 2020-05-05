from utils.DrealInterface import DrealInterface
from sympy import *
from sympy.logic.boolalg import to_nnf
from spec.conf import dRealJobs, dRealTimeout


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


class VC:

    def __init__(self, title, formulas, shouldBeSat = False):
        self.title = title
        #self.formulas = [ simplify_logic(f) for f in formulas ] # a list of formula from imprecise (easy to solve) to precise (hard to solve)
        #self.formulas = [ simplify(f) for f in formulas ]
        #self.formulas = [ to_nnf(f, simplify=False) for f in formulas ]
        self.formulas = [ to_nnf(f) for f in formulas ]
        self.sat = shouldBeSat
        self.model = None

    def __str__(self):
        sat = ""
        if self.sat:
            sat = "sat"
        else:
            sat = "unsat"
        return "VC(" + self.title + ", " + str(self.formulas) + "," + sat + ")"

    def _trivialOrSovler(self, formula, timeout, debug):
        trivialSat = True
        trivialUnsat = False
        for f in formula:
            if f == S.true:
                pass
            elif f == S.false:
                trivialUnsat = True
                trivialSat = False
            else:
                trivialSat = False
        if trivialUnsat:
            return False
        elif trivialSat:
            return True
        else:
            #TODO take out variables which are not important: the ones only appearing in bound cstrs (careful may be unsat)
            # split the formula into bound cstr and other cstr
            # look at the free symbols in other cstr
            # look in the bound cstr for var not needed var are sat
            # send the other cstr to the solver
            dr = DrealInterface(timeout = timeout, jobs = dRealJobs, debug = debug)
            res, model = dr.run(formula)
            self.model = model
            return res

    def hasModel(self):
        return self.model != None

    def modelStr(self):
        return "\n".join( str(k) + " = " + str (v) for k,v in self.model.items() )

    def discharge(self, timeout = dRealTimeout, debug = False):
        if debug:
            sat = ""
            if self.sat:
                sat = " (sat)"
            else:
                sat = " (unsat)"
            print("VC: " + self.title + sat)
        for f in self.formulas:
            #f2 = to_cnf(f)
            f3 = list(And.make_args(f))
            if debug:
                for f in f3:
                    print(f)
            res = self._trivialOrSovler(f3, timeout, debug)
            if res == None:
                return False
            elif res == self.sat:
                return True
        return False

