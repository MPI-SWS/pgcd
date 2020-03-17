from spec import *
from compatibility import *
from utils.geometry import *
from cart import Cart
from static_process import StaticProcess
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

class XpLaneParametricTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dy = 50

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
        s += " || ".join("{ (fpy > "+str(i*dy - dy/2+1)+") && (fpy < "+str(i*dy + dy/2-1)+") } x_"+str(i)+"_0" for i in range(0, n))
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

    def scenario(self, n, m, par, debug = False):
        print("## n =", n, ", m =", m, ", par =", par)
        w = self.world(n, m)
        if par:
            ch = self.choreo_par(n, m)
        else:
            ch = self.choreo_no_par(n, m)
        progs = { "Cart"+str(i): self.code(n,m) for i in range(0, n) }
        start = time.time()
        visitor = Projection()
        visitor.execute(ch, w, debug)
        chor = visitor.choreography
        vectorize(chor, w)
        end = time.time()
        print("Syntactic checks:", end - start)
        start = end
        checker = CompatibilityCheck(chor, w)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        checker.computePreds(debug)
        checker.generateCompatibilityChecks(debug)
        end = time.time()
        print("VC generation:", end - start)
        start = end
        print("#VC:", len(checker.vcs))
        for i in range(n, len(checker.vcs)): # XXX skip the one about the processes abstract FP
            vc = checker.vcs[i]
            #print("Checking VC", i, vc.title)
            if not vc.discharge(debug=debug):
                raise Exception(str(vc))
        end = time.time()
        print("VC solving:", end - start)
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
        print("refinement:", end - start)
        start = end
    
    def test_2_1(self, debug = False):
        self.scenario(2, 1, False, debug)
    
    def test_2_1_p(self, debug = False):
        self.scenario(2, 1, True, debug)
    
    def test_2_2(self, debug = False):
        self.scenario(2, 2, False, debug)
    
    def test_3_2_p(self, debug = False):
        self.scenario(2, 2, True, debug)
    
    def test_3_1(self, debug = False):
        self.scenario(3, 1, False, debug)
    
    def test_3_1_p(self, debug = False):
        self.scenario(3, 1, True, debug)
    
    def test_3_2(self, debug = False):
        self.scenario(3, 2, False, debug)
    
    def test_3_2_p(self, debug = False):
        self.scenario(3, 2, True, debug)
    
    def test_4_1(self, debug = False):
        self.scenario(4, 1, False, debug)
    
    def test_4_1_p(self, debug = False):
        self.scenario(4, 1, True, debug)
    
    def test_4_2(self, debug = False):
        self.scenario(4, 2, False, debug)
    
    def test_4_2_p(self, debug = False):
        self.scenario(4, 2, True, debug)
    
    def test_5_1(self, debug = False):
        self.scenario(5, 1, False, debug)
    
    def test_5_1_p(self, debug = False):
        self.scenario(5, 1, True, debug)
    
    def test_6_1(self, debug = False):
        self.scenario(6, 1, False, debug)
    
    def test_6_1_p(self, debug = False):
        self.scenario(6, 1, True, debug)
