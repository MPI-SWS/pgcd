from verification.spec.env import *
import verification.spec.conf as conf
from verification.choreography.compatibility import *
from verification.choreography.projection import Projection
import verification.choreography.vectorize
from verification.spec.component import World
from arm import Arm
from static_process import *

import unittest

def choreo():
    return ''' fixed =
        def start = (p1: wait(1), p2: wait(1)); x0
            x0 = end
        in [ true ] start
    '''

class ArmTest(unittest.TestCase):

#   def setUp(self):
#       self.defaultConf = conf.enableMPincludeFPCheck
#       conf.enableMPincludeFPCheck = False

#   def tearDown(self):
#       conf.enableMPincludeFPCheck = self.defaultConf

    def check(self, P1, P2):
        w = World()
        p1 = P1(w, 0)
        p2 = P2(w, 1)
        env = Env(w, [])
        ch = choreo()
        visitor = Projection()
        chor = visitor.parse(ch, env)
        verification.choreography.vectorize.vectorize(chor, w)
        checker = CompatibilityCheck(chor, w)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        checker.computePreds()
        checker.generateCompatibilityChecks()
        for i in range(0, len(checker.vcs)):
            vc = checker.vcs[i]
            if not vc.discharge():
                print(i, "inFP", vc.title)
                if vc.hasModel():
                    print(vc.modelStr())
                return False
        return True



    def test_01(self):
        tests = [
            #x1 y1 z1 r1 x2 y2 z2 r2 res
            (0, 0, 0, 1, 3, 0, 0, 1, True),
            (0, 0, 0, 1, 0, 3, 0, 1, True),
            (0, 0, 0, 1, 0, 0, 3, 1, True),
            (0, 0, 0, 1, 2, 2, 0, 1, True),
            (0, 0, 0, 1, 2, 0, 2, 1, True),
            (0, 0, 0, 1, 0, 2, 2, 1, True),
            (0, 0, 0, 1, 2, 0, 0, 1, False),
            (0, 0, 0, 1, 0, 2, 0, 1, False),
            (0, 0, 0, 1, 0, 0, 2, 1, False),
        ]
        for (x1, y1, z1, r1, x2, y2, z2, r2, res) in tests:
            p1 = lambda w, i: SphereProcess("p1", x1, y1, z1, r1, w, i)
            p2 = lambda w, i: SphereProcess("p2", x2, y2, z2, r2, w, i)
            self.assertTrue(self.check(p1, p2) == res)

    def test_02(self):
        tests = [
            #x1 y1 z1 r1 h1 x2 y2 z2 r2 h2 res
            (0, 0, 0, 1, 1, 3, 0, 0, 1, 1, True),
            (0, 0, 0, 1, 1, 0, 3, 0, 1, 1, True),
            (0, 0, 0, 1, 1, 0, 0, 3, 1, 1, True),
            (0, 0, 0, 1, 1, 2, 2, 0, 1, 1, True),
            (0, 0, 0, 1, 1, 2, 0, 2, 1, 1, True),
            (0, 0, 0, 1, 1, 0, 2, 2, 1, 1, True),
            (0, 0, 0, 1, 1, 2, 0, 0, 1, 1, False),
            (0, 0, 0, 1, 1, 0, 2, 0, 1, 1, False),
            (0, 0, 0, 1, 1, 0, 0, 2, 1, 1, True),
        ]
        for (x1, y1, z1, r1, h1, x2, y2, z2, r2, h2, res) in tests:
            p1 = lambda w, i: CylinderProcess("p1", x1, y1, z1, r1, h1, w, i)
            p2 = lambda w, i: CylinderProcess("p2", x2, y2, z2, r2, h2, w, i)
            self.assertTrue(self.check(p1, p2) == res)

if __name__ == '__main__':
    unittest.main()
