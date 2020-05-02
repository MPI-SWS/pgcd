import unittest
from spec.contract import *
from experiments_setups import World, DummyProcess

#TODO contract, vaccuous, refinement, composition, etc.

w = World()
dp1 = DummyProcess("DP1", w, 0)
dp2 = DummyProcess("DP2", w, 1)

class TestContract01(AssumeGuaranteeContract):

    def __init__(self, proc = dp1):
        super().__init__("TestContract01")
        self._p = proc

    def components(self):
        return {self._p};


class TestContract02(AssumeGuaranteeContract):

    def __init__(self, proc = dp1):
        super().__init__("TestContract02")
        self._p = proc

    def components(self):
        return {self._p};

    def preG(self):
        return S.true


class ContractTests(unittest.TestCase):
    
    def test_01(self):
        contract = TestContract01()
        vcs = contract.wellFormed()
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) > 0)

    def test_02(self):
        contract = TestContract02()
        vcs = contract.wellFormed()
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) == 0)

    def test_03(self):
        contract1 = TestContract01()
        contract2 = TestContract02()
        vcs = contract1.refines(contract2)
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) == 0)

    def test_04(self):
        contract1 = TestContract01()
        self.assertRaises(AssertionError, ComposedContract, contract1, contract1, dict())

    def test_05(self):
        contract1 = TestContract01()
        contract2 = TestContract02(dp2)
        contract3 = ComposedContract(contract1, contract2, dict())
        vcs = contract3.wellFormed()
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) > 0)

    def test_06(self):
        contract1 = TestContract02(dp1)
        contract2 = TestContract02(dp2)
        contract3 = ComposedContract(contract1, contract2, dict())
        vcs = contract3.wellFormed()
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) == 0)

if __name__ == '__main__':
    unittest.main()
