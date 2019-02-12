
import unittest
import parser
import datetime
from modelchecker import *
from experiments_setups import *
from experiments_setups_2 import *
import xp_handover_test
import xp_underpass_test

from itertools import *
def first_true(iterable, default=False, pred=None):
    return next(filter(pred, iterable), default)


class GlobalMcTests(unittest.TestCase):

    def test_01(self):
        start = datetime.datetime.now()
        programs = { "A": progFetchA(), "C": progFetchC() }
        prser = parser.Parser()
        parsed = dict( (name, prser.parse(txt)) for name, txt in programs.items() )
        world = cartAndArmWorld()
        arm = first_true(world.allProcesses(), None, lambda x: x.name() == "A")
        cart = first_true(world.allProcesses(), None, lambda x: x.name() == "C")
        annot = progFecthAnnot(arm, cart)
        mc = GlobalModelChecking(world, parsed, annot, debug = False)
        mc.check()
        end = datetime.datetime.now()
        delta = end - start
        print("RES fetch")
        print("RES elapsed: ", delta)
        print("RES #VC:", len(mc.vcs))

    def test_02(self):
        start = datetime.datetime.now()
        programs = { "Arm": progHandoverArm(), "Cart": progHandoverCart(), "Carrier": progHandoverCarrier() }
        prser = parser.Parser()
        parsed = dict( (name, prser.parse(txt)) for name, txt in programs.items() )
        world = xp_handover_test.xp2_world()
        arm = first_true(world.allProcesses(), None, lambda x: x.name() == "Arm")
        cart = first_true(world.allProcesses(), None, lambda x: x.name() == "Cart")
        carrier = first_true(world.allProcesses(), None, lambda x: x.name() == "Carrier")
        annot = progHandoverAnnot(arm, cart, carrier)
        mc = GlobalModelChecking(world, parsed, annot, debug = False)
        mc.check()
        end = datetime.datetime.now()
        delta = end - start
        print("RES handover")
        print("RES elapsed: ", delta)
        print("RES #VC:", len(mc.vcs))

    def test_03(self):
        start = datetime.datetime.now()
        programs = { "Arm": progUnderpassArm(), "Cart": progUnderpassCart(), "Carrier": progUnderpassCarrier() }
        prser = parser.Parser()
        parsed = dict( (name, prser.parse(txt)) for name, txt in programs.items() )
        world = xp_underpass_test.xp1_world()
        arm = first_true(world.allProcesses(), None, lambda x: x.name() == "Arm")
        cart = first_true(world.allProcesses(), None, lambda x: x.name() == "Cart")
        carrier = first_true(world.allProcesses(), None, lambda x: x.name() == "Carrier")
        annot = progUnderpassAnnot(arm, cart, carrier)
        mc = GlobalModelChecking(world, parsed, annot, debug = False)
        mc.check()
        end = datetime.datetime.now()
        delta = end - start
        print("RES underpass ")
        print("RES elapsed: ", delta)
        print("RES #VC:", len(mc.vcs))

    def test_04(self):
        start = datetime.datetime.now()
        programs = { "Arm": progTwistAndTurnArm(), "Cart": progTwistAndTurnCart(), "Carrier": progTwistAndTurnCarrier() }
        prser = parser.Parser()
        parsed = dict( (name, prser.parse(txt)) for name, txt in programs.items() )
        world = progTwistAndTurnWorld()
        arm = first_true(world.allProcesses(), None, lambda x: x.name() == "Arm")
        cart = first_true(world.allProcesses(), None, lambda x: x.name() == "Cart")
        carrier = first_true(world.allProcesses(), None, lambda x: x.name() == "Carrier")
        annot = progTwistAndTurnAnnot(arm, cart, carrier)
        mc = GlobalModelChecking(world, parsed, annot, debug = False)
        mc.check()
        end = datetime.datetime.now()
        delta = end - start
        print("RES twist and turn")
        print("RES elapsed: ", delta)
        print("RES #VC:", len(mc.vcs))
        pass

if __name__ == '__main__':
    unittest.main()
