from experiments_setups import XpTestHarness
from twistNturn_setup import *

def choreo_old():
    return '''twistAndTurn =
        def x0 = Cart -> Arm: OK(); x1
            x1 = (Arm: RotateAndGrab(0, -2.2689280275926285,  2.2689280275926285, 0,  2.0943951023931953, -0.3490658503988659),
                  Cart: Idle(),
                  Carrier: Idle()); x2
            x2 = Arm -> Cart: OK(); x3
            x3 = Cart -> Carrier: OK(); x4
            x4 = Cart -> Arm: OK(); x5
            x5 = (Arm: Rotate(0, 1.7443951023931953, -0.3490658503988659, 1.5707963267948966, -1.7443951023931953,  0.3490658503988659, 5),
                  Cart: SetAngleCart(0, 1.5707963267948966, 5),
                  Carrier: Swipe(0, 0, 0, 0.5, 1.5707963267948966, 5)); x6
            x6 = Carrier -> Cart: OK(); x7
            x7 = Cart -> Arm: OK(); x8
            x8 = (Arm: RotateAndPut(1.5707963267948966, -1.7443951023931953, 0.3490658503988659, 1.5707963267948966, -2.0943951023931953, 0.3490658503988659),
                  Cart: Idle(),
                  Carrier: Idle()); x9
            x9 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) ] x0
    '''


class XpTwist01Test(XpTestHarness):

    def test_twist_ecoop(self):
        ch = choreo_old()
        w = progTwistAndTurnWorld()
        contracts = []
        progs = { "Arm": progTwistAndTurnArm(),
                  "Cart": progTwistAndTurnCart(),
                  "Carrier": progTwistAndTurnCarrier() }
        self.check(ch, w, contracts, progs)

if __name__ == '__main__':
    unittest.main()

