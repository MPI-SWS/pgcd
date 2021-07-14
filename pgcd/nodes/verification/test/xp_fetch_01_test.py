from experiments_setups import XpTestHarness, cartAndArmWorld
from fetch_setup import progFetchA, progFetchC

def choreo_old():
    return '''Fetch  =
        def prepare0 = C -> A: fold(); prepare1
            prepare1 = (A: Fold(0, -2.2689280275926285, 2.2689280275926285), C: idle()); prepare2
            prepare2 = A -> C: folded(); go0
            go0 + go3 = go1
            go1 = [ sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1 ] go2 +
                  [ sqrt((C_x - 2)**2 + (C_y - 0)**2) <= 0.1 ] there0
            go2 = (A: idle(), C: moveFromTo(Pnt(0,0,0), Pnt(2,0,0), 0)); go3
            there0 = C -> A: grab(Pnt(2.29,0,0)); there1
            there1 = (A: Grab(Pnt(2.29,0,0), Pnt(2,0,0), 0), C: idle()); there2
            there2 = A -> C: grabbed(); there3
            there3 = C -> A: fold(); there4
            there4 = (A: Fold(), C: idle()); there5
            there5 = A -> C: folded(); return0
            return0 + return3 = return1
            return1 = [ sqrt((C_x - 0)**2 + (C_y - 0)**2) > 0.1 ] return2 +
                      [ sqrt((C_x - 0)**2 + (C_y - 0)**2) <= 0.1 ] done0
            return2 = (A: idle(), C: moveFromTo(Pnt(2,0,0), Pnt(0,0,0), 0)); return3
            done0 = C -> A: done(); done1
            done1 = end
        in [  (C_theta == 0) && (C_x == 0) && (C_y == 0) &&
              (A_a == 2.2689280275926285) && (A_b == -2.2689280275926285) && (A_c == 0)
           ] prepare0
    '''



class XpFetch01Test(XpTestHarness):

    def test_fetch_ecoop(self):
        ch = choreo_old()
        w = cartAndArmWorld()
        contracts = []
        progs = { "A": progFetchA(),
                  "C": progFetchC() }
        self.check(ch, w, contracts, progs)

if __name__ == '__main__':
    unittest.main()
