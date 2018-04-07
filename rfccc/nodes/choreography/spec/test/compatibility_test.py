import experiments_setups
from propagate_preds_chor import *
from sympy import *
from test.test import cartAndArmFetch, binSorting, armsHandover, ferry
from parser_chor import *
from vectorize_spec import *

import unittest

class CompatibilityCheckTest(unittest.TestCase):
    
    def test_fetch(self):
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
            self.assertTrue(vc.discharge(), str(vc))
        checker.vcs = []
        # compat
        checker.computePreds(False)
        checker.generateCompatibilityChecks()
        #TODO that example with bound
        #for vc in checker.vcs:
        #    self.assertTrue(vc.discharge(True), str(vc))

    def test_binsort(self):
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
            self.assertTrue(vc.discharge(), str(vc))
        checker.vcs = []
        # compat
        checker.computePreds(False)
        checker.generateCompatibilityChecks()
        for vc in checker.vcs:
            self.assertTrue(vc.discharge(), str(vc))

    def test_handover(self):
        world = experiments_setups.armsHandoverWorld()
        code = armsHandover()
        parser = ChoreographyParser()
        tree = parser.parse(code)
        vectorize(parser.state_to_node, world)
        checker = CompatibilityCheck(parser.state_to_node, tree, world)
        checker.localChoiceChecks()
        # total guards
        checker.generateTotalGuardsChecks()
        self.assertEqual(len(checker.vcs), 0)
        for vc in checker.vcs:
            self.assertTrue(vc.discharge(), str(vc))
        checker.vcs = []
        # compat
        checker.computePreds(False)
        checker.generateCompatibilityChecks()
        for vc in checker.vcs:
            self.assertTrue(vc.discharge(), str(vc))

    def test_ferry(self):
        world = experiments_setups.ferryWorld()
        code = ferry()
        parser = ChoreographyParser()
        tree = parser.parse(code)
        vectorize(parser.state_to_node, world)
        checker = CompatibilityCheck(parser.state_to_node, tree, world)
        checker.localChoiceChecks()
        # total guards
        checker.generateTotalGuardsChecks()
        self.assertEqual(len(checker.vcs), 0)
        for vc in checker.vcs:
            self.assertTrue(vc.discharge(), str(vc))
        checker.vcs = []
        # compat
        checker.computePreds(False)
        checker.generateCompatibilityChecks()
        for vc in checker.vcs:
            self.assertTrue(vc.discharge(), str(vc))

if __name__ == '__main__':
    unittest.main()
