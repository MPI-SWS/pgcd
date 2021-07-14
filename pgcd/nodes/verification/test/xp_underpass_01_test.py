from experiments_setups import XpTestHarness
from underpass_setup import *


#TODO Cart idle sync is funny, should be wait
def xp1_choreo():
    return ''' Bridge =
        def x0 = (Cart: idle(), Arm: setAngleTurntable(0, 1.5707963267948966, 10), Carrier: moveCart(0, 0, 0, 0.67, 10) ) ; x1
            x1 = (Cart: idle(), Arm: setAngleCantilever(-2.2689280275926285, 1.3962634015954636), Carrier: idle()) ; x2
            x2 = (Cart: idle(), Arm: setAngleAnchorPoint(2.2689280275926285, -0.3490658503988659), Carrier: idle()) ; x3
            x3 = Carrier -> Arm: OK(); x4
            x4 = (Cart: idle(), Arm: setAngleCantilever(1.3962634015954636, 2.0943951023931953), Carrier: idle()) ; x5
            x5 = (Cart: idle(), Arm: Grip(9.5), Carrier: idle()) ; x6
            x6 = (Cart: idle(), Arm: setAngleCantilever(2.0943951023931953, 0.87266462599716477), Carrier: idle()) ; x7
            x7 = Arm -> Carrier: OK(); x8
            x8 = (Cart: idle(), Arm: setAngleTurntable(1.5707963267948966, 0, 5), Carrier: setAngleCart(0.78539816339744828, 5)) ; x9
            x9 = (Cart: idle(), Arm: idle(), Carrier: moveCart(0.67, 0, rad(45), 0.5)) ; x10
            x10 = (Cart: idle(), Arm: idle(), Carrier: setAngleCart(rad(90))) ; x11
            x11 = (Cart: idle(), Arm: idle(), Carrier: strafeCart(1.023553390594, 0.353553390594, rad(90), 0.085)) ; x12
            x12 = Carrier -> Arm: OK(); x13
            x13 = (Cart: idle(), Arm: setAngleCantilever(0.87266462599716477, 2.0943951023931953), Carrier: idle()) ; x14
            x14 = (Cart: idle(), Arm: Grip(6), Carrier: idle()) ; x15
            x15 = Arm -> Carrier: OK(); x16
            x16 = (Cart: idle(), Arm: retractArm(0, 2.0943951023931953, -0.3490658503988659, 10), Carrier: moveCart(1.108553390594, 0.353553390594, rad(90), 0.7, 10)) ; x17
            x17 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

class XpUnderpass01(XpTestHarness):

    def test_underpass_ecoop(self):
        ch = xp1_choreo()
        w = xp1_world()
        contracts = []
        progs = { "Arm": xp1_arm(),
                  "Cart": xp1_cart(),
                  "Carrier": xp1_carrier() }
        self.check(ch, w, contracts, progs)

if __name__ == '__main__':
    unittest.main()
