from sympy import *
from sympy.vector import CoordSys3D
from spec.component import *
from spec.motion import *
from spec.time import *
import spec.conf
from spec.env import Env
from utils.geometry import *
from cart import *
from arm import *
from mpmath import mp
from compatibility import *
from refinement import *
from vectorize import *
from copy import deepcopy
from choreography.projection import Projection
import interpreter.parser as parser
import time

import unittest

# world: a trivial componenent (with optional mounting points as args)
class World(Component):

    def __init__(self, *mnts):
        super().__init__('World', None)
        f = spec.conf.worldFrame 
        self._frame = f
        self._mountingPoints = [f.orient_new_axis('world_mount_' + str(i), t, f.k, location= x * f.i + y * f.j + z * f.k) for i, (x,y,z,t) in enumerate(mnts)]
        self._mount = f

    def frame(self):
        return self._frame

    def mountingPoint(self, index):
        if index < len(self._mountingPoints) :
            return self._mountingPoints[index]
        else:
            return self._frame

class DummyProcess(Process):

    def __init__(self, name, parent, index):
        super().__init__(name, parent, index)
        Idle(self)

    def frame(self):
        return self._parent.frame()

    def outputVariables(self):
        return [Symbol(self.name() + "_x"), Symbol(self.name() + "_y"), Symbol(self.name() + "_z")]

    def abstractResources(self, *arg):
        return S.false

    def mountingPoint(self, index):
        return self.frame()


def cartAndArmWorld():
    w = World()
    c = Cart("C", w)
    a = Arm("A", c)
    return w

def armsHandoverWorld():
    w = World( (-0.28,0,0,0), (0.28,0,0,mp.pi) )
    a1 = Arm("A1", w, 0)
    a2 = Arm("A2", w, 1)
    return w

def binSortingWorld():
    w = World( (0,0,0,mp.pi/2), (0.3,0,0,-mp.pi/2) )
    a1 = Arm("A", w, 0)
    a2 = Arm("B", w, 1)
    return w

def ferryWorld():
    w = World( (-1,-0.5,0,mp.pi/2), (1,-0.5,0,mp.pi/2) )
    a1 = Arm("A1", w, 0)
    a2 = Arm("A2", w, 1)
    c = Cart("C", w, 2)
    return w


def mkDummyWorld(*cmpNames):
    w = World()
    for i, name in enumerate(cmpNames):
        c = DummyProcess(name, world, i)
    return w



class XpTestHarness(unittest.TestCase):
    
    def setUp(self):
        self.defaultConf = spec.conf.enableMPincludeFPCheck
        spec.conf.enableMPincludeFPCheck = False
        self.defaultPrecision = spec.conf.dRealPrecision
        spec.conf.dRealPrecision = 0.01

    def tearDown(self):
        spec.conf.enableMPincludeFPCheck = self.defaultConf
        spec.conf.dRealPrecision = self.defaultPrecision


    def check(self, ch, w, contracts, progs, debug):
        start = time.time()
        start0 = start
        env = Env(w, contracts)
        visitor = Projection()
        visitor.execute(ch, env, debug)
        chor = visitor.choreography
        if debug:
            print("parsed", chor)
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
        failed = []
        for i in range(0, len(checker.vcs)):
            vc = checker.vcs[i]
            print("Checking VC", i, vc.title)
            if not vc.discharge(debug=debug):
                print("Failed")
                failed.append((i,vc))
                print(vc)
                if vc.hasModel():
                    print(vc.modelStr())
                else: 
                    print("Timeout")
        for (i,vc) in failed:
            print("Failed:", i, vc.title)
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
        print("total time:", end - start0)
        self.assertTrue(failed == [])
