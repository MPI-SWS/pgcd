import unittest
from verification.spec.contract import *
from verification.spec.time import timifyFormula
from verification.spec.component import World
from experiments_setups import DummyProcess
from verification.utils.geometry import cube

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

class TestContract03(AssumeGuaranteeContract):

    def __init__(self, proc, x, y, z):
        super().__init__("TestContract03")
        self._p = proc
        self.x = x
        self.y = y
        self.z = z

    def components(self):
        return {self._p};

    def preG(self):
        return S.true

    def fp(self, point):
        f = self.frame();
        lbl = f.origin.locate_new(self.name + "_lbl", self.x * f.i + self.y * f.j + self.z * f.k);
        ufr = f.origin.locate_new(self.name + "_lbl", (self.x + 1) * f.i + (self.y + 1) * f.j + (self.z + 1) * f.k);
        return cube(f, lbl, ufr, point);

    def preFP(self, point):
        return self.fp(point);

    def invFP(self, point):
        return timifyFormula(self.allVariables(), self.fp(point));

    def postFP(self, point):
        return self.fp(point);

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
        self.assertTrue(len(wrong) == 3) # FP are not disjoint

    def test_07(self):
        contract = TestContract03(dp1, 0, 0, 0)
        vcs = contract.wellFormed()
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) == 0)

    def test_08(self):
        contract1 = TestContract03(dp1, 0, 0, 0)
        contract2 = TestContract03(dp2, 0, 0, 0)
        vcs = contract1.checkCollision(contract2, dict(), w.frame())
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) == 3)


    def test_09(self):
        contract1 = TestContract03(dp1, -1, 0, 0)
        contract2 = TestContract03(dp2, 1, 0, 0)
        vcs = contract1.checkCollision(contract2, dict(), w.frame())
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) == 0)

    def test_10(self):
        contract1 = TestContract03(dp1, -1, 0, 0)
        contract2 = TestContract03(dp2, 1, 0, 0)
        contract3 = ComposedContract(contract1, contract2, dict())
        vcs = contract3.wellFormed()
        wrong = [ vc for vc in vcs if not(vc.discharge())]
        self.assertTrue(len(wrong) == 0)

if __name__ == '__main__':
    unittest.main()
