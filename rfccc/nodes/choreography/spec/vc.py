from DrealInterface import DrealInterface
from sympy import *

class VC:

    def __init__(self, title, formulas, shouldBeSat = False):
        self.title = title
        #self.formulas = [ simplify_logic(f) for f in formulas ] # a list of formula from imprecise (easy to solve) to precise (hard to solve)
        self.formulas = formulas
        self.sat = shouldBeSat

    def __str__(self):
        sat = ""
        if self.sat:
            sat = "sat"
        else:
            sat = "unsat"
        return "VC(" + self.title + ", " + str(self.formulas) + "," + sat + ")"

    def discharge(self, debug = False):
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
            dr = DrealInterface(debug = debug)
            res, model = dr.run(f3)
            if res == None:
                return False
            elif res == self.sat:
                return True
        return False

