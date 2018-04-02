import experiments_setups
from propagate_preds_chor import *
from sympy import *
from test.test import cartAndArmFetch, binSorting
from parser_chor import *

import unittest

class CompatibilityCheckTest(unittest.TestCase):
    
    def test_01(self):
        world = experiments_setups.cartAndArmWorld()
        code = cartAndArmFetch()
        parser = ChoreographyParser()
        tree = parser.parse(code)
        checker = CompatibilityCheck(parser.state_to_node, parser.start_state, world)
        checker.localChoiceChecks()

    def test_02(self):
        world = experiments_setups.binSortingWorld()
        code = binSorting()
        parser = ChoreographyParser()
        tree = parser.parse(code)
        checker = CompatibilityCheck(parser.state_to_node, parser.start_state, world)
        checker.localChoiceChecks()

if __name__ == '__main__':
    unittest.main()
