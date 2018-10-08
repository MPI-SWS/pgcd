
import unittest
import parser
from global_mc import *
from experiments_setups import *
from spin_test import progFetchA, progFetchC

class GlobalMcTests(unittest.TestCase):

    def test_01(self):
        programs = { "A": progFetchA(), "C": progFetchC() }
        prser = parser.Parser()
        parsed = dict( (name, prser.parse(txt)) for name, txt in programs.items() )
        world = cartAndArmWorld()
        annot = {}
        mc = GlobalModelChecking(world, parsed, annot, debug = True)
        mc.check()

if __name__ == '__main__':
    unittest.main()
