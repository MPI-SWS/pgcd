from spec.env import *
import spec.conf 
from compatibility import *
from projection import Projection
from arm import Arm
from static_process import PointProcess
from vectorize import *
from mpmath import mp
from experiments_setups import World

import unittest


def world(x, y, z, a, b, c):
    # mounting pts:  x      y     z        θ     comp
    w = World(  (    0,     0,    0,       0), # arm
                (    0,     0,    0,       0)) # point
    #arm   = Arm("arm", w, 0, -2.2689280275926285, -2.2689280275926285, 0) #folded toward the back
    arm   = Arm("arm", w, 0, a, b, c) #folded toward the back
    point = PointProcess("point", x, y, z, w, 1)
    return w


def choreo1():
    return ''' fixed =
        def start = (arm: Wait(1), point: Wait(1)); x0
            x0 = end
        in [ (arm_a == 0) && (arm_b == 0) && (arm_c == 0) ] start
          
    '''

def choreo2(a_lb, a_ub, b_lb, b_ub, c_lb, c_ub):
    a = "(arm_a >= "+str(a_lb)+") && (arm_a <= "+str(a_ub)+")"
    b = "(arm_b >= "+str(b_lb)+") && (arm_b <= "+str(b_ub)+")"
    c = "(arm_c >= "+str(c_lb)+") && (arm_c <= "+str(c_ub)+")"
    init = a + " && " + b + " && " + c
    return ''' bounds =
        def start = (arm: Wait(1), point: Wait(1)); x0
            x0 = end
        in [ ''' + init + " ] start"

def code():
    return '''
Wait(1);
    '''

class ArmTest(unittest.TestCase):

    def setUp(self):
        self.defaultConf = spec.conf.enableMPincludeFPCheck
        spec.conf.enableMPincludeFPCheck = False

    def tearDown(self):
        spec.conf.enableMPincludeFPCheck = self.defaultConf

    def check(self, ch, x, y, z, a, b, c, debug = False):
        w = world(x, y, z, a, b, c)
        env = Env(w, [])
        visitor = Projection()
        visitor.execute(ch, env, debug)
        chor = visitor.choreography
        print(chor.id, x, y, z, a, b, c)
        vectorize(chor, w)
        checker = CompatibilityCheck(chor, w)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        checker.computePreds(debug)
        checker.generateCompatibilityChecks(debug)
        for i in range(0, len(checker.vcs)):
            vc = checker.vcs[i]
            if not vc.discharge(debug=debug):
                print(i, "inFP", vc.title)
                if vc.hasModel():
                    print(vc.modelStr())
                return False
        return True


    def test_fixed_positions(self, debug = False):
        pi = mp.pi
        tests = [
            #ch             x       y       z       a       b       c       result (False == in FP)
            (choreo1(),     0,      0,      0,      0,      0,      0,      False),
            (choreo1(),     0,      0,      0.1,    0,      0,      0,      False),
            (choreo1(),     0,      0,      0.5,    0,      0,      0,      False),
            (choreo1(),     0,      0,      -1,     0,      0,      0,      True),
            (choreo1(),     0,      0,      -0.1,   0,      0,      0,      True),
            (choreo1(),     0,      0,      -0.02,  0,      0,      0,      True),
            (choreo1(),     0,      0,      0.62,   0,      0,      0,      True), #FIXME I would expect the reach to be FP 0.55 but it gets to 0.61 !?!
            (choreo1(),     0,      0,      0.60,   pi/2,   0,      0,      True),
            (choreo1(),     0,      0,      0.50,   pi/2,   0,      0,      False),
            (choreo1(),     0,      -0.2,   0.45,   pi/2,   0,      0,      True),
            (choreo1(),     0,      0.2,    0.45,   pi/2,   0,      0,      True),
            (choreo1(),     -0.2,   0,      0.45,   pi/2,   0,      0,      True),
            (choreo1(),     0.2,    0,      0.45,   pi/2,   0,      0,      True),
            (choreo1(),     0,      -0.12,  0.52,   0,      0,      0,      True), #FIXME again, should be lower !?
            (choreo1(),     0,      -0.12,  0.52,   pi/2,   0,      0,      True),
            (choreo1(),     0,      0.12,   0.52,   pi/2,   0,      0,      True),
            (choreo1(),     -0.12,  0,      0.52,   pi/2,   0,      0,      True),
            (choreo1(),     0.12,   0,      0.52,   pi/2,   0,      0,      False),
            (choreo1(),     0,      -0.12,  0.52,   -pi/2,  0,      0,      True),
            (choreo1(),     0,      0.12,   0.52,   -pi/2,  0,      0,      True),
            (choreo1(),     -0.12,  0,      0.52,   -pi/2,  0,      0,      False),
            (choreo1(),     0.12,   0,      0.52,   -pi/2,  0,      0,      True),
            (choreo1(),     0,      -0.12,  0.52,   0,      pi/2,   0,      True),
            (choreo1(),     0,      0.12,   0.52,   0,      pi/2,   0,      True),
            (choreo1(),     -0.12,  0,      0.52,   0,      pi/2,   0,      True),
            (choreo1(),     0.12,   0,      0.52,   0,      pi/2,   0,      True),
            (choreo1(),     0,      -0.12,  0.52,   0,      -pi/2,  0,      True),
            (choreo1(),     0,      0.12,   0.52,   0,      -pi/2,  0,      True),
            (choreo1(),     -0.12,  0,      0.52,   0,      -pi/2,  0,      True),
            (choreo1(),     0.12,   0,      0.52,   0,      -pi/2,  0,      True),
            (choreo1(),     0,      -0.32,  0.32,   0,      pi/2,   0,      True),
            (choreo1(),     0,      0.32,   0.32,   0,      pi/2,   0,      True),
            (choreo1(),     -0.32,  0,      0.32,   0,      pi/2,   0,      True),
            (choreo1(),     0.32,   0,      0.32,   0,      pi/2,   0,      False),
            (choreo1(),     0,      -0.32,  0.32,   0,      -pi/2,  0,      True),
            (choreo1(),     0,      0.32,   0.32,   0,      -pi/2,  0,      True),
            (choreo1(),     -0.32,  0,      0.32,   0,      -pi/2,  0,      False),
            (choreo1(),     0.32,   0,      0.32,   0,      -pi/2,  0,      True),
            (choreo1(),     0.39,   0,      0.32,   0,      pi/2,   0,      True), #FIXME again, londer than expected
            (choreo1(),     -0.39,  0,      0.32,   0,      -pi/2,  0,      True),
        ]
        for (ch, x, y, z, a, b, c, res) in tests:
            self.assertTrue(self.check(ch, x, y, z, a, b, c, debug) == res)


    def test_a_free(self, debug = False):
        pi = mp.pi
        ch = choreo2(-pi, pi, 0, 0, 0, 0)
        tests = [
            #ch      x       y       z       a       b       c       result (False == in FP)
            (ch,     0,      0,      0,      0,      0,      0,      False),
            (ch,     0,      0,      0.1,    0,      0,      0,      False),
            (ch,     0,      0,      0.5,    0,      0,      0,      False),
            (ch,     0,      0,      -1,     0,      0,      0,      True),
            (ch,     0,      0,      -0.1,   0,      0,      0,      True),
            (ch,     0,      0,      0.65,   0,      0,      0,      True), # TODO check the length
#           (ch,     0,      -0.12,  0.52,   0,      0,      0,      True), #FIXME Timeout
#           (ch,     0,      0.12,   0.52,   0,      0,      0,      True), #FIXME Timeout
            (ch,     0.12,   0,      0.52,   0,      0,      0,      False),
#           (ch,     -0.12,  0,      0.52,   0,      0,      0,      False), #FIXME Timeout
        ]
        for (ch, x, y, z, a, b, c, res) in tests:
            self.assertTrue(self.check(ch, x, y, z, a, b, c, debug) == res)


if __name__ == '__main__':
    unittest.main()
