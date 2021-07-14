from verification.choreography.compatibility import *
from verification.utils.geometry import *
from verification.choreography.refinement import *
from verification.choreography.vectorize import *
from verification.spec.component import World
from verification.choreography.projection import Projection
from mpmath import mp
from static_process import CubeProcess
from copy import deepcopy
import interpreter.parser as parser
import time
from experiments_setups import XpTestHarness

import unittest


def world():
    # mounting pts:  x      y     z        θ     comp
    w = World(  (   -1,     0,    0,       0), # producer
                (    0,     0,    0,       0)) # sensor
    # specs
    producer  = CubeProcess("producer", 0, 0, 0, 0, 0.2, 0.2, 0.05, w, 0)
    sensor    = CubeProcess("sensor", 0, 0, 0, 0, 0.2, 0.2, 0.2, w, 1)
    return w

def choreo1():
    return ''' Sorting =
        def start = (producer: wait(1), sensor: wait(1)); x0
            x0 = { fpx > -0.4 } x1 || # sensor
                 { fpx < -0.6 } x2    # producer
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



class FrameShiftTest(XpTestHarness):

    def test_01(self):
        w = world()
        ch = choreo1()
        contracts = []
        progs = { "sensor": code_sensor(),
                  "producer": code_producer()}
        self.check(ch, w, contracts, progs)

if __name__ == '__main__':
    unittest.main()
