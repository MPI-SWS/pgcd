from spec.component import Cube
from spec.contract import StaticContract, AContract, GContract
from spec.time import DurationSpec
from utils.geometry import *
from sympy import symbols, Eq, And, S
from cart import Cart
from cart import CartSquare
from arm import Arm
from mpmath import mp
from experiments_setups import World, XpTestHarness, cartAndArmWorld
from experiments_setups_2 import progFetchA, progFetchC

def choreo_old():
    return '''Fetch  =
        def prepare0 = C -> A: fold(); prepare1
            prepare1 = (A: Fold(0, -2.2689280275926285, 2.2689280275926285), C: Idle()); prepare2
            prepare2 = A -> C: folded(); go0
            go0 + go3 = go1
            go1 = [ sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1 ] go2 +
                  [ sqrt((C_x - 2)**2 + (C_y - 0)**2) <= 0.1 ] there0
            go2 = (A: Idle(), C: MoveFromTo(Pnt(0,0,0), Pnt(2,0,0), 0)); go3
            there0 = C -> A: grab(Pnt(2.29,0,0)); there1
            there1 = (A: Grab(Pnt(2.29,0,0), Pnt(2,0,0), 0), C: Idle()); there2
            there2 = A -> C: grabbed(); there3
            there3 = C -> A: fold(); there4
            there4 = (A: Fold(), C: Idle()); there5
            there5 = A -> C: folded(); return0
            return0 + return3 = return1
            return1 = [ sqrt((C_x - 0)**2 + (C_y - 0)**2) > 0.1 ] return2 +
                      [ sqrt((C_x - 0)**2 + (C_y - 0)**2) <= 0.1 ] done0
            return2 = (A: Idle(), C: MoveFromTo(Pnt(2,0,0), Pnt(0,0,0), 0)); return3
            done0 = C -> A: done(); done1
            done1 = end
        in [  (C_theta == 0) && (C_x == 0) && (C_y == 0) &&
              (A_a == 2.2689280275926285) && (A_b == -2.2689280275926285) && (A_c == 0)
           ] prepare0
    '''


#TODO grab spec, fold spec
# contract in the frame of A for the collision ...

def choreo_new():
    return '''Fetch  =
        def prepare0 = C -> A: fold(); split1
            split1 = @GContract(C, C_y == 0 && C_x == 0 && C_theta == 0, fpz < 0.1) c1 ||
                     @AContract(A, C_y == 0 && C_x == 0 && C_theta == 0, fpz > 0.1) a1
            c1 = (C: Idle()); c2
            a1= (A: Fold(0, -2.2689280275926285, 2.2689280275926285)); a2
            c2 || a2 = prepare2
            prepare2 = A -> C: folded(); split2
            split2 = @GContract(C, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz < 0.1) c3 ||
                     @AContract(A, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz > 0.1) a3
            c3 + go3 = go1
            go1 = [ sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1 ] go2 +
                  [ sqrt((C_x - 2)**2 + (C_y - 0)**2) <= 0.1 ] there0
            go2 = (C: MoveFromTo(Pnt(0,0,0), Pnt(2,0,0), 0)); go3
            a3 = (A: Idle()); a3b
            a3b || there0 = there1
            there1 = C -> A: grab(Pnt(2.29,0,0)); split3
            split3 = @GContract(C, C_y == 0 && C_x == 2 && C_theta == 0, fpz < 0.1 && ((fpx-2)**2 + fpy**2) < 0.26**2) c4 ||
                     @AContract(A, C_y == 0 && C_x == 2 && C_theta == 0, fpz > 0.1 || ((fpx-2)**2 + fpy**2) > 0.27**2) a4
            c4 = (C: Idle()); c5
            a4 = (A: Grab(Pnt(2.29,0,0), Pnt(2,0,0), 0)); a5
            c5 || a5 = there2
            there2 = A -> C: grabbed(); there3
            there3 = C -> A: fold(); split4
            split4 = @GContract(C, C_y == 0 && C_x == 2 && C_theta == 0, fpz < 0.1 && ((fpx-2)**2 + fpy**2) < 0.26**2) c6 ||
                     @AContract(A, C_y == 0 && C_x == 2 && C_theta == 0, fpz > 0.1 || ((fpx-2)**2 + fpy**2) > 0.27**2) a6
            c6 = (C: Idle()); c7
            a6 = (A: Fold()); a7
            c7 || a7 = there5
            there5 = A -> C: folded(); split5
            split5 = @GContract(C, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz < 0.1) c8 ||
                     @AContract(A, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz > 0.1) a8
            c8 + return3 = return1
            return1 = [ sqrt((C_x - 0)**2 + (C_y - 0)**2) > 0.1 ] return2 +
                      [ sqrt((C_x - 0)**2 + (C_y - 0)**2) <= 0.1 ] done0
            return2 = (C: MoveFromTo(Pnt(2,0,0), Pnt(0,0,0), 0)); return3
            a8 = (A: Idle()); a8b
            a8b || done0 = done1
            done1 = C -> A: done(); done2
            done2 = end
        in [  (C_theta == 0) && (C_x == 0) && (C_y == 0) &&
              (A_a == 2.2689280275926285) && (A_b == -2.2689280275926285) && (A_c == 0)
           ] prepare0
    '''

class XpfetchTest(XpTestHarness):

    def test_fetch(self, debug = False):
        ch = choreo_old()
        w = cartAndArmWorld() 
        contracts = []
        progs = { "A": progFetchA(),
                  "C": progFetchC() }
        self.check(ch, w, contracts, progs, debug)

    def test_fetch_new(self, debug = False):
        ch = choreo_new()
        w = cartAndArmWorld() 
        contracts = [GContract, AContract]
        progs = { "A": progFetchA(),
                  "C": progFetchC() }
        self.check(ch, w, contracts, progs, debug)


if __name__ == '__main__':
    unittest.main()
        
