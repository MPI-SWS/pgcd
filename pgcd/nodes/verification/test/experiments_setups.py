import spec.conf
from sympy import *
from sympy.vector import CoordSys3D
from spec.component import *
from spec.motion import *
from spec.time import *
from spec.env import Env
from utils.geometry import *
from cart import *
from arm import *
from mpmath import mp
from choreography.compatibility import *
from choreography.refinement import *
from choreography.vectorize import *
from copy import deepcopy
from choreography.projection import Projection
import interpreter.parser as parser
import time

import unittest
import logging

log = logging.getLogger("XP")

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


    def check(self, ch, w, contracts, progs):
        start = time.time()
        start0 = start
        env = Env(w, contracts)
        visitor = Projection()
        chor = visitor.parse(ch, env)
        log.debug("parsed\n%s", chor)
        vectorize(chor, w)
        end = time.time()
        log.info("Syntactic checks: %s", end - start)
        start = end
        checker = CompatibilityCheck(chor, w)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        checker.computePreds()
        checker.generateCompatibilityChecks()
        end = time.time()
        log.info("VC generation: %s", end - start)
        start = end
        log.info("#VC: %s", len(checker.vcs))
        failed = []
        for i in range(0, len(checker.vcs)):
            vc = checker.vcs[i]
            log.info("Checking VC %s %s", i, vc.title)
            if not vc.discharge():
                log.warning("Failed VC %s", i)
                failed.append((i,vc))
                log.debug("%s", vc)
                if vc.hasModel():
                    log.debug("%s", vc.modelStr())
                else:
                    log.debug("Timeout")
        for (i,vc) in failed:
            log.info("Failed: %s %s", i, vc.title)
        end = time.time()
        log.info("VC solving: %s", end - start)
        start = end
        processes = w.allProcesses()
        for p in processes:
            visitor.choreography = deepcopy(chor)
            proj = visitor.project(p.name(), p)
            prser = parser.Parser()
            prog = prser.parse(progs[p.name()])
            ref = Refinement(prog, proj)
            if not ref.check():
                raise Exception("Refinement: " + p.name())
        end = time.time()
        log.info("refinement: %s", end - start)
        log.info("total time: %s", end - start0)
        self.assertTrue(failed == [])
