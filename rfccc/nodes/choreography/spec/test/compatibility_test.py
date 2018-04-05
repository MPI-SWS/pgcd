import experiments_setups
from propagate_preds_chor import *
from sympy import *
from test.test import cartAndArmFetch, binSorting
from parser_chor import *
from vectorize_spec import *

import unittest

class CompatibilityCheckTest(unittest.TestCase):
    
    def test_01(self):
        world = experiments_setups.cartAndArmWorld()
        code = cartAndArmFetch()
        parser = ChoreographyParser()
        tree = parser.parse(code)
        vectorize(parser.state_to_node, world)
        #for n in parser.state_to_node.values():
        #    print(str(n))
    
    def test_02(self):
        world = experiments_setups.cartAndArmWorld()
        code = cartAndArmFetch()
        parser = ChoreographyParser()
        tree = parser.parse(code)
        vectorize(parser.state_to_node, world)
        checker = CompatibilityCheck(parser.state_to_node, tree, world)
        checker.localChoiceChecks()
        # total guards
        checker.generateTotalGuardsChecks()
        self.assertEqual(len(checker.vcs), 2)
        for vc in checker.vcs:
            self.assertEqual(vc.discharge(), True)
        checker.vcs = []
        # compat
        checker.computePreds(False)
        checker.generateCompatibilityChecks()
        #for vc in checker.vcs:
        #    self.assertEqual(vc.discharge(), True)

    def test_03(self):
        world = experiments_setups.binSortingWorld()
        code = binSorting()
        parser = ChoreographyParser()
        tree = parser.parse(code)
        vectorize(parser.state_to_node, world)
        checker = CompatibilityCheck(parser.state_to_node, tree, world)
        checker.localChoiceChecks()
        # total guards
        checker.generateTotalGuardsChecks()
        self.assertEqual(len(checker.vcs), 4)
        for vc in checker.vcs:
            self.assertEqual(vc.discharge(), True)
        checker.vcs = []
        # compat
        checker.computePreds(True)
        checker.generateCompatibilityChecks()
        for vc in checker.vcs:
            self.assertEqual(vc.discharge(True), True)

if __name__ == '__main__':
    unittest.main()
