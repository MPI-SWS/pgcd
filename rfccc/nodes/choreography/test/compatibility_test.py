import experiments_setups
from compatibility import *
from sympy import *
from parser_test import cartAndArmFetch, binSorting, armsHandover, ferry, ferry1
from parser_chor import *
from vectorize_chor import *

import unittest

class CompatibilityCheckTest(unittest.TestCase):

    checkVCs = True

    def test_fetch(self):
        world = experiments_setups.cartAndArmWorld()
        code = cartAndArmFetch()
        parser = ChoreographyParser()
        tree = parser.parse(code, world)
        vectorize(tree, world)
        checker = CompatibilityCheck(tree, world)
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
        if CompatibilityCheckTest.checkVCs:
            for vc in checker.vcs:
                self.assertTrue(vc.discharge(), str(vc))

    def test_binsort(self):
        world = experiments_setups.binSortingWorld()
        code = binSorting()
        parser = ChoreographyParser()
        tree = parser.parse(code, world)
        vectorize(tree, world)
        checker = CompatibilityCheck(tree, world)
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
        if CompatibilityCheckTest.checkVCs:
            for vc in checker.vcs:
                self.assertTrue(vc.discharge(), str(vc))

    def test_handover(self):
        world = experiments_setups.armsHandoverWorld()
        code = armsHandover()
        parser = ChoreographyParser()
        tree = parser.parse(code, world)
        vectorize(tree, world)
        checker = CompatibilityCheck(tree, world)
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
        if CompatibilityCheckTest.checkVCs:
            for vc in checker.vcs:
                self.assertTrue(vc.discharge(), str(vc))

    def test_ferry(self):
        world = experiments_setups.ferryWorld()
        code = ferry()
        parser = ChoreographyParser()
        tree = parser.parse(code, world)
        vectorize(tree, world)
        checker = CompatibilityCheck(tree, world)
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
        if CompatibilityCheckTest.checkVCs:
            for vc in checker.vcs:
                self.assertTrue(vc.discharge(), str(vc))

    def test_ferry1(self):
        world = experiments_setups.ferryWorld()
        code = ferry1()
        parser = ChoreographyParser()
        tree = parser.parse(code, world)
        vectorize(tree, world)
        checker = CompatibilityCheck(tree, world)
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
        if CompatibilityCheckTest.checkVCs:
            for vc in checker.vcs:
                self.assertTrue(vc.discharge(), str(vc))

if __name__ == '__main__':
    unittest.main()
