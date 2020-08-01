
import spec.conf
from spec.component import Cube
from spec.contract import StaticContract
from spec.time import DurationSpec
from utils.geometry import *
from sympy import symbols, Eq, And, S
from mpmath import mp
from experiments_setups import XpTestHarness 
from experiments_setups_2 import progTwistAndTurnArm, progTwistAndTurnCart, progTwistAndTurnCarrier, progTwistAndTurnWorld

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

def choreo_new():
    return '''twistAndTurn =
        def x0 = Cart -> Arm: OK(); x1 
            x1 = (Arm: RotateAndGrab(0, -2.2689280275926285,  2.2689280275926285, 0,  2.0943951023931953, -0.3490658503988659), Cart: Idle(), Carrier: Idle()); x2
            x2 = Arm -> Cart: OK(); x3
            x3 = Cart -> Carrier: OK(); x4 
            x4 = Cart -> Arm: OK(); x5 
            x5 = { fpx**2 + fpy**2 < 0.35**2 || (fpz > 0.18 && fpx**2 + fpy**2 > 0.35**2) } ca0 ||
                 { fpz < 0.17 && fpx**2 + fpy**2 > 0.36**2 } cb0
            ca0 = (Arm: Rotate(0, 1.7443951023931953, -0.3490658503988659, 1.5707963267948966, -1.7443951023931953,  0.3490658503988659, 5), Cart: SetAngleCart(0, 1.5707963267948966, 5)); ca1
            cb0 = (Carrier: Swipe(0, 0, 0, 0.5, 1.5707963267948966, 5)); cb1
            ca1 || cb1 = x6
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

class CartContract(StaticContract):

    def __init__(self, component, fp, duration = DurationSpec(1, 1, False)):
        cx, cy, ct = symbols("Cart_x Cart_y Cart_theta")
        g = And(Eq(cx, 0), Eq(cy, 0), ct >= 0, ct <= 1.58)
        super().__init__("CartContract", {component}, S.true, g, fp, duration)

class ArmContract(StaticContract):
    
    def __init__(self, component, fp, duration = DurationSpec(1, 1, False)):
        cx, cy, ct = symbols("Cart_x Cart_y Cart_theta")
        a = And(Eq(cx, 0), Eq(cy, 0), ct >= 0, ct <= 1.58)
        super().__init__("ArmContract", {component}, a, S.true, fp, duration)

def choreo_new1():
    return '''twistAndTurn =
        def x0 = Cart -> Arm: OK(); x1 
            x1 = (Arm: RotateAndGrab(0, -2.2689280275926285,  2.2689280275926285, 0,  2.0943951023931953, -0.3490658503988659), Cart: Idle(), Carrier: Idle()); x2
            x2 = Arm -> Cart: OK(); x3
            x3 = Cart -> Carrier: OK(); x4 
            x4 = Cart -> Arm: OK(); x5 
            x5 = @CartContract(Cart, fpz < 0.09 && fpx**2 + fpy**2 < 0.3**2) ca0 ||
                 @ArmContract(Arm,  (fpz > 0.1 && fpx**2 + fpy**2 < 0.35**2) || (fpz > 0.18 && fpx**2 + fpy**2 > 0.35**2) ) a0 ||
                 { fpz < 0.17 && fpx**2 + fpy**2 > 0.36**2 } cb0
            a0 = (Arm: Rotate(0, 1.7443951023931953, -0.3490658503988659, 1.5707963267948966, -1.7443951023931953,  0.3490658503988659)); a1
            a1 = (Arm: Idle()); a2
            ca0 = (Cart: SetAngleCart(0, 1.5707963267948966, 2)); ca1
            ca1 = (Cart: Idle()); ca2
            cb0 = (Carrier: Swipe(0, 0, 0, 0.5, 1.5707963267948966, 5)); cb1
            a2 || ca2 || cb1 = x6
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


class XpfetchTest(XpTestHarness):

    def test_fetch(self):
        ch = choreo_old()
        w = progTwistAndTurnWorld()
        contracts = []
        progs = { "Arm": progTwistAndTurnArm(),
                  "Cart": progTwistAndTurnCart(),
                  "Carrier": progTwistAndTurnCarrier() }
        self.check(ch, w, contracts, progs)

    def test_fetch_new(self):
        ch = choreo_new()
        w = progTwistAndTurnWorld() 
        contracts = [CartContract, ArmContract]
        progs = { "Arm": progTwistAndTurnArm(),
                  "Cart": progTwistAndTurnCart(),
                  "Carrier": progTwistAndTurnCarrier() }
        self.check(ch, w, contracts, progs)


if __name__ == '__main__':
    unittest.main()
        
