from spec.component import Process
from compatibility import *
from utils.geometry import *
from arm import Arm
from franka import FrankaEmikaPanda
from static_process import StaticProcess
from refinement import *
from vectorize import *
from mpmath import mp
from experiments_setups import World
from copy import deepcopy
from choreography.projection import Projection
import interpreter.parser as parser
import time

import unittest

# FIXME not yet finished 

# A "microbenchmark" to emphasize the benefit of having the parallel composition as part of the spec.
# Consider a series of arm in a line and handing over objects between each others.

class XpHandoverChainParametricTest(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dx = 0.5
    
    def world(self, n):
        # mounting pts:   x  y  z  θ
        w = World( *[ (i*self.dx, 0, 0, 0) for i in range(0, n) ] )
        # cmpts:
        for i in range(0, n):
            Arm("Arm" + str(i), w, i) #TODO option for Panda arm
        return w

    def grab(self, i, code, left):
        x = i * self.dx
        if left:
            x -= self.dx / 2
        else:
            x += self.dx / 2
        y = 0
        z = 0.1
        if code: # from m to mm
            x *= 1000
            y *= 1000
            z *= 1000
        s = "MoveTo("
        if not code:
            s += "Pnt("
        s += str(x) +", "+str(y)+", "+str(z)+")"
        if not code:
            s += ")"
        return s
    
    def grabLeft(self, i, code):
        return self.grab(i, code, True)
    
    def grabRight(self, i, code):
        return self.grab(i, code, False)

    def choreo_no_par(self, n):
        def mkTuple(i, motion):
            s = "("
            s += ", ".join("Arm"+str(j)+": "+("Idle()" if j != i else motion) for j in range(0,n))
            s += ")"
            return s
        s = "Chain =\ndef x0 = "
        for i in range(0, n):
            x1 = "x"+str(4*i+1)
            x2 = "x"+str(4*i+2)
            x3 = "x"+str(4*i+3)
            x4 = "x"+str(4*i+4)
            s += mkTuple(i, self.grabLeft(i, False))
            s += "; "+x1+"\n  "+x1+" = "
            s += mkTuple(i, self.grabRight(i, False))
            s += "; "+x2+"\n  "+x2+" = "
            s += mkTuple(i, "Fold()")
            s += "; "+x3+"\n  "+x3+" = "
            if i != n-1:
                s += "Arm"+str(i)+" -> Arm"+str(i+1)+": Ok(); "+x4+"\n  "+x4+" = "
        for i in range(1, n):
            x1 = "x"+str(4*n+i)
            s += "Arm"+str(n-i)+" -> Arm"+str(n-i-1)+": Ok(); "+x1+"\n  "+x1+" = "
        s += 'end\nin [ '
        s += ' &&\n    '.join( "(Arm"+str(i)+"_a == 0) && (Arm"+str(i)+"_b == 0) && (Arm"+str(i)+"_c == 0)" for i in range(0, n))
        s += ' ] x0'
        return s
    
    def choreo_par(self, n):
        s = "Chain =\ndef x0 = "
        ...
    
    def code(self, n, i):
        pre = "Arm"+str(i-1)
        cur = "Arm"+str(i)
        nex = "Arm"+str(i+1)
        s = ""
        if i != 0:
            s += "receive("+pre+", Idle) { case Ok() => skip; }\n" # wait for start
        x = self.dx / 2 * 100
        y = 0
        z = 0.1 * 100
        s += self.grabLeft(0, True) + ";\n"
        s += self.grabRight(0, True) + ";\n"
        s += "Fold();\n"
        if i != n-1:
            s += "send("+nex+", Ok);\n" # signal next
            s += "receive("+nex+", Idle) { case Ok() => skip; }\n" # wait for end
        if i != 0:
            s += "send("+pre+", Ok);\n"
        return s
    
    def scenario(self, n, par, debug = False):
        print("## n =", n, ", par =", par)
        w = self.world(n)
        if par:
            ch = self.choreo_par(n)
        else:
            ch = self.choreo_no_par(n)
        print(ch)
        progs = {}
        for i in range(0, n):
            a = "Arm"+str(i)
            c = self.code(n,i)
            progs[a] = c
            print(a)
            print(c)
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
        for i in range(0, len(checker.vcs)):
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
    
#   def test_1(self, debug = False):
#       self.scenario(1, False, debug)
   
#   def test_2(self, debug = False):
#       self.scenario(2, False, debug)
   
#   def test_3(self, debug = False):
#       self.scenario(3, False, debug)
    
    def test_1_p(self, debug = False):
        self.scenario(1, True, debug)
