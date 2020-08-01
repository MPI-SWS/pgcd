from experiments_setups import XpTestHarness
from handover_setup import *

# TODO dReal timeout

def xp2_choreo_1():
    return ''' Handover =
        def x0 = (Cart: MoveCart(0, 0, 0, 0.3, 5), Arm: Idle(), Carrier: MoveCart(0, 0, 0, 0.5, 5) ) ; x1
            x1 = Carrier -> Cart: OK(); x2
            x2 = Cart -> Arm: OK(); x3
            x3 = (Cart: Idle(), Arm: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953), Carrier: Idle() ) ; x4
            x4 = (Cart: Idle(), Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659), Carrier: Idle() ) ; x5
            x5 = (Cart: Idle(), Arm: Grip(9.5), Carrier: Idle() ) ; x6
            x6 = (Cart: Idle(), Arm: RetractArm(0,2.0943951023931953,-0.3490658503988659), Carrier: Idle() ) ; x7
            x7 = Arm -> Cart: OK(); x8
            x8 = Cart -> Carrier: OK(); x9
            x9 = (Cart: MoveCart(0.3, 0, 0, -0.3, 5), Arm: Idle(), Carrier: MoveCart(0.5, 0, 0, -0.5, 5) ) ; x10
            x10 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''


class XpHandover01Test(XpTestHarness):

    def test_handover_ecoop(self):
        ch = xp2_choreo_1()
        contracts = []
        w = xp2_world()
        progs = { "Arm": xp2_arm(),
                  "Cart": xp2_cart(),
                  "Carrier": xp2_carrier() }
        self.check(ch, w, contracts, progs)

#TODO need proper AG contracts
#   def test_handover_3(self):
#       self.handover(xp2_choreo_3())

if __name__ == '__main__':
    unittest.main()
