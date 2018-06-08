import experiments_setups
from compatibility import *
from sympy import *
from parser_test import cartAndArmFetch, binSorting, armsHandover, ferry, ferry1
from parser_chor import *
from vectorize_chor import *
import minimize
import normalization_chor

import unittest

class CompatibilityCheckTest(unittest.TestCase):

    checkVCs = True
    #checkVCs = False

    artificial_nodes_counter = -1

    def get_artificial_name(self):
        CompatibilityCheckTest.artificial_nodes_counter += 1
        return '__x__' + str(CompatibilityCheckTest.artificial_nodes_counter)

    def compat(self, code, world, expectedNbrGuardVCs, checkVCs, debug = False):
        parser = ChoreographyParser()
        tree = parser.parse(code, world)
        vectorize(tree, world)
        if tree.hasParallel():
            state_to_node = tree.mk_state_to_node()
            CompatibilityCheckTest.artificial_nodes_counter = -1
            #print("before normal")
            #print(tree)
            normalization_chor.removeForkJoin(self, tree, state_to_node, debug)
            #print("after normal")
            #print(tree)
        checker = CompatibilityCheck(tree, world)
        checker.localChoiceChecks()
        # total guards
        checker.generateTotalGuardsChecks()
        self.assertEqual(len(checker.vcs), expectedNbrGuardVCs)
        for vc in checker.vcs:
            self.assertTrue(vc.discharge(), str(vc))
        checker.vcs = []
        # compat
        checker.computePreds(debug)
        checker.generateCompatibilityChecks()
        if checkVCs:
            for vc in checker.vcs:
                self.assertTrue(vc.discharge(), str(vc))

    def test_fetch(self):
        world = experiments_setups.cartAndArmWorld()
        code = cartAndArmFetch()
        self.compat(code, world, 2, CompatibilityCheckTest.checkVCs)

    def test_binsort(self):
        world = experiments_setups.binSortingWorld()
        code = binSorting()
        self.compat(code, world, 4, CompatibilityCheckTest.checkVCs)

    def test_handover(self):
        world = experiments_setups.armsHandoverWorld()
        code = armsHandover()
        self.compat(code, world, 0, CompatibilityCheckTest.checkVCs)

    def test_ferry(self):
        world = experiments_setups.ferryWorld()
        code = ferry()
        self.compat(code, world, 0, CompatibilityCheckTest.checkVCs)

    def test_ferry1(self):
        world = experiments_setups.ferryWorld()
        code = ferry1()
        self.compat(code, world, 0, CompatibilityCheckTest.checkVCs)

if __name__ == '__main__':
    unittest.main()
