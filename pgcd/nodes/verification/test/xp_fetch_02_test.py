from experiments_setups import XpTestHarness, cartAndArmWorld
from fetch_setup import progFetchA, progFetchC
from verification.spec.contract import AContract, GContract

def choreo_new():
    return '''Fetch  =
        def prepare0 = C -> A: fold(); split1
            split1 = @GContract(C, C_y == 0 && C_x == 0 && C_theta == 0, fpz < 0.1) c1 ||
                     @AContract(A, C_y == 0 && C_x == 0 && C_theta == 0, fpz > 0.1) a1
            c1 = (C: idle()); c2
            a1= (A: Fold(0, -2.2689280275926285, 2.2689280275926285)); a2
            c2 || a2 = prepare2
            prepare2 = A -> C: folded(); split2
            split2 = @GContract(C, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz < 0.1) c3 ||
                     @AContract(A, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz > 0.1) a3
            c3 + go3 = go1
            go1 = [ sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1 ] go2 +
                  [ sqrt((C_x - 2)**2 + (C_y - 0)**2) <= 0.1 ] there0
            go2 = (C: moveFromTo(Pnt(0,0,0), Pnt(2,0,0), 0)); go3
            a3 = (A: idle()); a3b
            a3b || there0 = there1
            there1 = C -> A: grab(Pnt(2.29,0,0)); split3
            split3 = @GContract(C, C_y == 0 && C_x == 2 && C_theta == 0, fpz < 0.1 && ((fpx-2)**2 + fpy**2) < 0.26**2) c4 ||
                     @AContract(A, C_y == 0 && C_x == 2 && C_theta == 0, fpz > 0.1 || ((fpx-2)**2 + fpy**2) > 0.27**2) a4
            c4 = (C: idle()); c5
            a4 = (A: Grab(Pnt(2.29,0,0), Pnt(2,0,0), 0)); a5
            c5 || a5 = there2
            there2 = A -> C: grabbed(); there3
            there3 = C -> A: fold(); split4
            split4 = @GContract(C, C_y == 0 && C_x == 2 && C_theta == 0, fpz < 0.1 && ((fpx-2)**2 + fpy**2) < 0.26**2) c6 ||
                     @AContract(A, C_y == 0 && C_x == 2 && C_theta == 0, fpz > 0.1 || ((fpx-2)**2 + fpy**2) > 0.27**2) a6
            c6 = (C: idle()); c7
            a6 = (A: Fold()); a7
            c7 || a7 = there5
            there5 = A -> C: folded(); split5
            split5 = @GContract(C, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz < 0.1) c8 ||
                     @AContract(A, C_y >= -0.008 && C_y <= 0.008 && C_x >= -0.008 && C_x <= 2.005 && C_theta == 0,  fpz > 0.1) a8
            c8 + return3 = return1
            return1 = [ sqrt((C_x - 0)**2 + (C_y - 0)**2) > 0.1 ] return2 +
                      [ sqrt((C_x - 0)**2 + (C_y - 0)**2) <= 0.1 ] done0
            return2 = (C: moveFromTo(Pnt(2,0,0), Pnt(0,0,0), 0)); return3
            a8 = (A: idle()); a8b
            a8b || done0 = done1
            done1 = C -> A: done(); done2
            done2 = end
        in [  (C_theta == 0) && (C_x == 0) && (C_y == 0) &&
              (A_a == 2.2689280275926285) && (A_b == -2.2689280275926285) && (A_c == 0)
           ] prepare0
    '''

def contracts():
    return [GContract, AContract]


class XpFetch02Test(XpTestHarness):

    def test_fetch_oopsla(self):
        ch = choreo_new()
        w = cartAndArmWorld()
        ct = contracts()
        progs = { "A": progFetchA(),
                  "C": progFetchC() }
        self.check(ch, w, ct, progs)


if __name__ == '__main__':
    unittest.main()
