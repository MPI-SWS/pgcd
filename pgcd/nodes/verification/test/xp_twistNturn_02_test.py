from experiments_setups import XpTestHarness
from twistNturn_setup import *

def choreo_new():
    return '''twistAndTurn =
        def x0 = Cart -> Arm: OK(); x1
            x1 = (Arm: RotateAndGrab(0, -2.2689280275926285,  2.2689280275926285, 0,  2.0943951023931953, -0.3490658503988659), Cart: idle(), Carrier: idle()); x2
            x2 = Arm -> Cart: OK(); x3
            x3 = Cart -> Carrier: OK(); x4
            x4 = Cart -> Arm: OK(); x5
            x5 = { fpx**2 + fpy**2 < 0.35**2 || (fpz > 0.18 && fpx**2 + fpy**2 > 0.35**2) } ca0 ||
                 { fpz < 0.17 && fpx**2 + fpy**2 > 0.36**2 } cb0
            ca0 = (Arm: rotate(0, 1.7443951023931953, -0.3490658503988659, 1.5707963267948966, -1.7443951023931953,  0.3490658503988659, 5), Cart: setAngleCart(0, 1.5707963267948966, 5)); ca1
            cb0 = (Carrier: swipe(0, 0, 0, 0.5, 1.5707963267948966, 5)); cb1
            ca1 || cb1 = x6
            x6 = Carrier -> Cart: OK(); x7
            x7 = Cart -> Arm: OK(); x8
            x8 = (Arm: RotateAndPut(1.5707963267948966, -1.7443951023931953, 0.3490658503988659, 1.5707963267948966, -2.0943951023931953, 0.3490658503988659),
                  Cart: idle(),
                  Carrier: idle()); x9
            x9 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) ] x0
    '''

class XpTwist02Test(XpTestHarness):

    def test_twist_oopsla(self):
        ch = choreo_new()
        w = progTwistAndTurnWorld()
        contracts = [CartContract, ArmContract]
        progs = { "Arm": progTwistAndTurnArm(),
                  "Cart": progTwistAndTurnCart(),
                  "Carrier": progTwistAndTurnCarrier() }
        self.check(ch, w, contracts, progs)


if __name__ == '__main__':
    unittest.main()

