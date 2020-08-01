from experiments_setups import XpTestHarness
from handover_setup import *

def xp2_choreo_2():
    return ''' Handover =
        def x0 = { fpx < 0.558  } ca0 ||
                 { fpx > 0.558  } c0
            ca0 = (Cart: MoveCart(0, 0, 0,0.3, 5), Arm: Idle()) ; ca1
            c0 = (Carrier: MoveCart(0, 0, 0, 0.5, 5) ) ; c1
            ca1 || c1 = x1
            x1 = Carrier -> Cart: OK(); x2
            x2 = Cart -> Arm: OK(); x3
            x3 = { (fpx > 0.558) && (fpz < 0.167) } cr3 ||
                 { (fpx < 0.558) || (fpz > 0.167) } ct3
            ct3 = (Cart: Idle(), Arm: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953)) ; ct4
            ct4 = (Cart: Idle(), Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659)) ; ct5
            ct5 = (Cart: Idle(), Arm: Grip(9.5) ) ; ct6
            ct6 = (Cart: Idle(), Arm: RetractArm(0,2.0943951023931953,-0.3490658503988659)) ; ct7
            cr3 = (Carrier: Idle()) ; cr7
            ct7 || cr7 = x7
            x7 = Arm -> Cart: OK(); x8
            x8 = Cart -> Carrier: OK(); x9
            x9 = { fpx < 0.558 } ca9 ||
                 { fpx > 0.558 } c9
            ca9 = (Cart: MoveCart(0.3, 0, 0, -0.3, 5), Arm: Idle()) ; ca10
            c9 = (Carrier: MoveCart(0.5, 0, 0, -0.5, 5) ) ; c10
            ca10 || c10 = x10
            x10 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

class XpHandover02Test(XpTestHarness):

    def test_handover_ooplsa(self):
        ch = xp2_choreo_2()
        contracts = []
        w = xp2_world()
        progs = { "Arm": xp2_arm(),
                  "Cart": xp2_cart(),
                  "Carrier": xp2_carrier() }
        self.check(ch, w, contracts, progs)

if __name__ == '__main__':
    unittest.main()
