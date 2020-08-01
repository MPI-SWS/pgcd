from spec.component import Cube
from spec.contract import StaticContract
from spec.time import DurationSpec
from utils.geometry import *
from experiments_setups import World
from cart import *
from arm import *
from sympy import symbols, Eq, And, S
from mpmath import mp

def progTwistAndTurnWorld():
    w = World( (0,0,0,0), (0.5, 0, 0, mp.pi) )
    cart = Cart("Cart", w, 0)
    arm = Arm("Arm", cart)
    carrier = CartSquare("Carrier", w, 1)
    return w

def progTwistAndTurnArm():
    return '''
    A0: receive(Arm, Idle ){
        case OK() => skip;
    }
    A1: RotateAndGrab( 0, -2.2689280275926285,  2.2689280275926285,
                       0,  2.0943951023931953, -0.3490658503988659);
    send( Cart, OK, 1.0 );
    A2: receive(Arm, Idle ){
        case OK() => skip;
    }
    A3: Rotate( 0,                   1.7443951023931953, -0.3490658503988659,
                1.5707963267948966, -1.7443951023931953,  0.3490658503988659);
    A4: receive(Arm, Idle ){
        case OK() => skip;
    }
    A5: RotateAndPut( 1.5707963267948966, -1.7443951023931953, 0.3490658503988659,
                      1.5707963267948966, -2.0943951023931953, 0.3490658503988659);
    '''

def progTwistAndTurnCart():
    return '''
    send( Arm, OK, 1.0 );
    B0: receive(Arm, Idle ){
        case OK() => skip;
    }
    send( Carrier, OK, 1.0 );
    send( Arm, OK, 1.0 );
    B1: SetAngleCart( 1.5707963267948966 );
    B2: receive(Carrier, Idle ){
        case OK() => skip;
    }
    send( Arm, OK, 1.0 );
    B3: Idle;
    '''

def progTwistAndTurnCarrier():
    return '''
    C0: receive(Cart, Idle ){
        case OK() => skip;
    }
    C1: Swipe(0, 0, 0, 0.5, 1.5707963267948966);
    send( Cart, OK, 1.0 );
    C2: Idle;
    '''

def progTwistAndTurnAnnot(arm, car, carrier):
    Arm_a, Arm_b, Arm_c = sp.symbols('Arm_a, Arm_b, Arm_c')
    Cart_x, Cart_y, Cart_theta = sp.symbols('Cart_x, Cart_y, Cart_theta')
    Carrier_x, Carrier_y, Carrier_theta = sp.symbols('Carrier_x, Carrier_y, Carrier_theta')
    return {
      "A0": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, arm.minAngleAB), sp.Eq(Arm_c, 0)),
      "A1": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, arm.minAngleAB), sp.Eq(Arm_c, 0)),
      "A2": sp.And(sp.Eq(Arm_a, -0.3490658503988659), sp.Eq(Arm_b,  1.7443951023931953), sp.Eq(Arm_c, 0)),
      "A3": sp.And(sp.Eq(Arm_a, -0.3490658503988659), sp.Eq(Arm_b,  1.7443951023931953), sp.Eq(Arm_c, 0)),
      "A4": sp.And(sp.Eq(Arm_a,  0.3490658503988659), sp.Eq(Arm_b, -1.7443951023931953), sp.Eq(Arm_c, 1.5707963267948966)),
      "A5": sp.And(sp.Eq(Arm_a,  0.3490658503988659), sp.Eq(Arm_b, -1.7443951023931953), sp.Eq(Arm_c, 1.5707963267948966)),
      "B0": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B1": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B2": sp.And(sp.Eq(Cart_theta, 1.5707963267948966), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B3": sp.And(sp.Eq(Cart_theta, 1.5707963267948966), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "C0": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0), sp.Eq(Carrier_y, 0)),
      "C1": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0), sp.Eq(Carrier_y, 0)),
      "C2": sp.And(sp.Eq(Carrier_theta, 1.5707963267948966), sp.Eq(Carrier_x, 0.5), sp.Eq(Carrier_y, 0.5))
    }

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
