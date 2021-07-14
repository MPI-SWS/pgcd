from verification.spec.contract import FpContract
from verification.spec.env import *
import verification.spec.conf as conf
from verification.choreography.compatibility import *
from verification.utils.geometry import *
from verification.choreography.refinement import *
from verification.choreography.vectorize import *
from verification.spec.component import World
from verification.choreography.projection import Projection
from static_process import CubeProcess
from mpmath import mp
from copy import deepcopy
import interpreter.parser as parser
import time

import unittest


def world():
    # mounting pts:  x      y     z        θ     comp
    w = World(  (   -1,     0,    0,       0), # producer
                (    0,     0,    0,       0)) # sensor
    # specs
    producer  = CubeProcess("producer", 0, 0, 0, 0, 0.2, 0.2, 0.05, w, 0)
    sensor    = CubeProcess("sensor", 0, 0, 0, 0, 0.2, 0.2, 0.2, w, 1)
    return w


class Xless(FpContract):
    
    def __init__(self, component, expr):
        super().__init__("Xless", {component}, conf.worldFrame,
                          Symbol('x'), Symbol('y'), Symbol('z'),
                          Symbol('x') < expr)


class Xmore(FpContract):
    
    def __init__(self, component, expr):
        super().__init__("Xmore", {component}, conf.worldFrame,
                          Symbol('x'), Symbol('y'), Symbol('z'),
                          Symbol('x') > expr)

def choreo1():
    return ''' test =
        def start = (producer: wait(1), sensor: wait(1)); x0
            x0 = @Xmore(sensor, -0.4) x1 || # sensor
                 @Xless(producer, -0.6) x2  # producer
            x1 = (sensor: wait(1)) ; x3
            x2 = (producer: wait(1)) ; x4
            x3 || x4 = x5
            x5 = end
        in [ true ] start
    '''

def code_sensor():
    return '''
wait(1);
wait(1);
    '''

def code_producer():
    return '''
wait(1);
wait(1);
    '''



class ContractParsingTest(unittest.TestCase):

    def test_01(self):
        w = world()
        env = Env(w, [Xless, Xmore])
        ch = choreo1()
        progs = { "sensor": code_sensor(),
                  "producer": code_producer()}
        start = time.time()
        visitor = Projection()
        chor = visitor.parse(ch, env)
        vectorize(chor, w)
        end = time.time()
        print("Syntactic checks:", end - start)
        start = end
        checker = CompatibilityCheck(chor, w)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        checker.computePreds()
        checker.generateCompatibilityChecks()
        end = time.time()
        print("VC generation:", end - start)
        start = end
        print("#VC:", len(checker.vcs))
        failed = []
        for i in range(0, len(checker.vcs)):
            vc = checker.vcs[i]
            print("Checking VC", i, vc.title)
            if not vc.discharge():
                failed.append(vc)
                print("Failed")
                print(vc)
                if vc.hasModel():
                    print(vc.modelStr())
        end = time.time()
        print("VC solving:", end - start)
        self.assertTrue(failed == [])
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
        print("refinement:", end - start)
        start = end

if __name__ == '__main__':
    unittest.main()
