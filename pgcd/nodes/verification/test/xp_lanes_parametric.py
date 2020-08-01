from compatibility import *
from utils.geometry import *
from cart import Cart
from refinement import *
from vectorize import *
from experiments_setups import World
from copy import deepcopy
from choreography.projection import Projection
import interpreter.parser as parser
import time

import unittest

# A "microbenchmark" to emphasize the benefit of having the parallel composition as part of the spec.
# This example has n robots. Each robot is moving in its own lane. All the lanes are parallel to each other.
# TODO find the right dy so that the contraints used the precise FP rather than the abstract one.

class XpLaneParametricTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dy = 0.65

    def world(self, n, m):
        # mounting pts:   x  y  z  θ
        w = World( *[ (0, i*self.dy, 0, 0) for i in range(0, n) ] )
        # cmpts:
        for i in range(0, n):
            Cart("Cart" + str(i), w, i)
        return w

    def choreo_no_par(self, n, m):
        assert m > 0 and n > 0
        s = "Lane =\ndef x0 = "
        for j in range(1, m+1):
            s += '('
            s += ', '.join("Cart" + str(i) + ": MoveCart(0, 0, 0, 0.5, 5)" for i in range(0, n))
            s += ') ; x'+str(2*j-1)+'\n  x'+str(2*j-1)+' = ('
            s += ', '.join("Cart" + str(i) + ": MoveCart(0.5, 0, 0,-0.5, 5)" for i in range(0, n))
            s += ') ; x'+str(2*j)+'\n  x'+str(2*j)+' = '
        s += 'end\nin [ '
        s += ' &&\n    '.join( "(Cart"+str(i)+"_x == 0) && (Cart"+str(i)+"_y == 0) && (Cart"+str(i)+"_theta == 0)" for i in range(0, n))
        s += ' ] x0'
        return s

    def choreo_par(self, n, m):
        assert m > 0 and n > 0
        dy = self.dy
        s = "Lane =\ndef x0 = "
        s += " || ".join("{ (fpy > "+str(i*dy - dy/2)+") && (fpy < "+str(i*dy + dy/2)+") } x_"+str(i)+"_0" for i in range(0, n))
        s += "\n"
        for i in range(0, n):
            for j in range(0, m):
                s += "  x_"+str(i)+"_"+str(2*j  )+" = (Cart"+str(i)+": MoveCart(0, 0, 0, 0.5, 5)) ; x_"+str(i)+"_"+str(2*j+1)+"\n"
                s += "  x_"+str(i)+"_"+str(2*j+1)+" = (Cart"+str(i)+": MoveCart(0.5, 0, 0,-0.5, 5)) ; x_"+str(i)+"_"+str(2*j+2)+"\n"
        s += '  '
        s += ' || '.join("x_"+str(i)+"_"+str(2*m) for i in range(0, n))
        s += ' = x1\n  x1 = end\nin [ '
        s += ' &&\n    '.join( "(Cart"+str(i)+"_x == 0) && (Cart"+str(i)+"_y == 0) && (Cart"+str(i)+"_theta == 0)" for i in range(0, n))
        s += ' ] x0'
        return s
    
    def code(self, n, m):
        return ''.join('moveCart( 500); moveCart(-500);' for i in range(0, m))

    def scenario(self, n, m, par, debug = False, printCSV = False):
        if not printCSV:
            print("## n =", n, ", m =", m, ", par =", par)
        w = self.world(n, m)
        if par:
            ch = self.choreo_par(n, m)
        else:
            ch = self.choreo_no_par(n, m)
        if debug:
            print(ch)
        progs = { "Cart"+str(i): self.code(n,m) for i in range(0, n) }
        start = time.time()
        start0 = start
        visitor = Projection()
        visitor.execute(ch, w, debug)
        chor = visitor.choreography
        vectorize(chor, w)
        end = time.time()
        time_syntax = end - start
        if not printCSV:
            print("  Syntactic checks:", time_syntax)
        start = end
        checker = CompatibilityCheck(chor, w)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        checker.computePreds(debug)
        checker.generateCompatibilityChecks(debug)
        end = time.time()
        time_vc_gen = end - start
        if not printCSV:
            print("  VC generation:", time_vc_gen)
        start = end
        if not printCSV:
            print("  #VC:", len(checker.vcs))
        for i in range(0, len(checker.vcs)):
            vc = checker.vcs[i]
            #print("Checking VC", i, vc.title)
            if not vc.discharge(debug=debug):
                print(n, ",", m, ",", par, ",", time_syntax, ",", len(checker.vcs), ",", time_vc_gen, ", TO, NA, TO")
                return
#               if vc.hasModel():
#                   raise Exception(str(vc) + "\n" + vc.modelStr())
#               else: 
#                   raise Exception(str(vc))
        end = time.time()
        time_vc_solve = end - start
        if not printCSV:
            print("  VC solving:", time_vc_solve)
        start = end
        processes = w.allProcesses()
        for p in processes:
            visitor.choreography = deepcopy(chor)
            proj = visitor.project(p.name(), p, debug)
            prser = parser.Parser()
            prog = prser.parse(progs[p.name()])
            ref = Refinement(prog, proj, debug)
            if not ref.check():
                raise Exception("Refinement: " + p.name())
        end = time.time()
        time_refine = end - start
        if not printCSV:
            print("  refinement:", time_refine)
        time_total = end - start0
        if not printCSV:
            print("totat time:", time_total)
        else:
            print(n, ",", m, ",", par, ",", time_syntax, ",", len(checker.vcs), ",", time_vc_gen, ",", time_vc_solve, ",", time_refine, ",", time_total)

    def test_range(self):
        print("n, m, par, time_syntax, nbr_vc, time_vc_gen, time_vc_solve, time_refine, time_total")
        for n in range(2, 11):
            for m in range(1, 2):
                self.scenario(n, m, False, printCSV = True)
        for n in range(2, 30):
            for m in range(1, 2):
                self.scenario(n, m, True, printCSV = True)
