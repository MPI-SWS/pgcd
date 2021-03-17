import experiments_setups
from compatibility import *
from sympy import *

import unittest

class PredTrackerTests(unittest.TestCase):

    def test_01(self):
        w = experiments_setups.cartAndArmWorld()
        t = ProcessesPredicatesTracker(w.allProcesses())
        self.assertEqual(t.pred(), S.true)

    def test_02(self):
        w = experiments_setups.cartAndArmWorld()
        t = ProcessesPredicatesTracker(w.allProcesses())
        pred = Eq(Symbol('C_x'), Integer(0))
        t.addPred(pred)
        self.assertEqual(t.pred(), pred)

    def test_03(self):
        w = experiments_setups.cartAndArmWorld()
        t = ProcessesPredicatesTracker(w.allProcesses())
        pred = Eq(Symbol('C_x'), Integer(0))
        t.addPred(pred)
        t.relaxVariables([Symbol('C_y')])
        self.assertEqual(t.pred(), pred)
        t.relaxVariables([Symbol('C_x')])
        self.assertEqual(t.pred(), S.true)

    def test_04(self):
        w = experiments_setups.cartAndArmWorld()
        t = ProcessesPredicatesTracker(w.allProcesses())
        self.assertEqual(t == t, True)

    def test_05(self):
        w = experiments_setups.cartAndArmWorld()
        t1 = ProcessesPredicatesTracker(w.allProcesses())
        t2 = t1.copy()
        self.assertEqual(t1 == t2, True)
        t2.addPred(Eq(Symbol('C_x'), Integer(0)))
        self.assertEqual(t1.contains(t2), True)
        self.assertEqual(t2.contains(t1), False)
        t3 = t2.copy()
        self.assertEqual(t2 == t3, True)
        t3.addPred(Eq(Symbol('C_x'), Integer(0)))
        self.assertEqual(t2 == t3, True)
        t3.addPred(Eq(Symbol('C_y'), Integer(0)))
        self.assertEqual(t2.contains(t3), True)
        self.assertEqual(t3.contains(t2), False)

    def test_06(self):
        w = experiments_setups.cartAndArmWorld()
        t = ProcessesPredicatesTracker(w.allProcesses())
        pred1 = Eq(Symbol('C_x'), Integer(0))
        pred2 = Eq(Symbol('A_a'), Integer(0))
        pred = And(pred1, pred2)
        t.addFormula(pred)
        self.assertEqual(t.pred(), pred)

    def test_07(self):
        w = experiments_setups.cartAndArmWorld()
        t1 = ProcessesPredicatesTracker(w.allProcesses())
        t2 = ProcessesPredicatesTracker(w.allProcesses())
        pred1 = Eq(Symbol('C_x'), Integer(0))
        pred2 = Eq(Symbol('A_a'), Integer(0))
        pred3 = Eq(Symbol('C_y'), Integer(0))
        t1.addPred(pred1)
        t1.addPred(pred2)
        t2.addPred(pred3)
        t2.merge(t1)
        #print(t2.pred())
        self.assertEqual(t2.contains(t1), True)
        self.assertEqual(t1.contains(t2), False)

if __name__ == '__main__':
    unittest.main()
