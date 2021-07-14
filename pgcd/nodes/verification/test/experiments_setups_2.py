import sympy as sp
from verification.spec.component import World
from mpmath import mp
from cart import *
from arm import *

# Handover

def progHandoverArm():
    return '''
    A0: receive(Cart, Idle ){
        case OK() => skip;
    }
    A1: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953);
    A2: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659);
    A3: Grip( 9.5 );
    A4: RetractArm;
    send( Cart, OK, 1.0 );
    A5: Idle;
    '''

def progHandoverCart():
    return '''
    B0: MoveCart(0, 0, 0, 0.3);
    B1: receive(Carrier, Idle ){
        case OK() => skip;
    }
    send( Arm, OK, 1.0 );
    B2: receive(Arm, Idle ){
        case OK() => skip;
    }
    send( Carrier, OK, 1.0 );
    B3: MoveCart(0.3, 0, 0, -0.3);
    '''

def progHandoverCarrier():
    return '''
    C0: MoveCart(0, 0, 0, 0.5);
    send( Cart, OK, 1.0 );
    C1: receive(Cart, Idle ){
        case OK() => skip;
    }
    C2: MoveCart(0.5, 0, 0, -0.5);
    '''

def progHandoverAnnot(arm, cart, carrier):
    Arm_a, Arm_b, Arm_c = sp.symbols('Arm_a, Arm_b, Arm_c')
    Cart_x, Cart_y, Cart_theta = sp.symbols('Cart_x, Cart_y, Cart_theta')
    Carrier_x, Carrier_y, Carrier_theta = sp.symbols('Carrier_x, Carrier_y, Carrier_theta')
    return {
      "A0": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, arm.minAngleAB), sp.Eq(Arm_c, 0)),
      "A1": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, arm.minAngleAB), sp.Eq(Arm_c, 0)),
      "A2": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 2.0943951023931953), sp.Eq(Arm_c, 0)),
      "A3": sp.And(sp.Eq(Arm_a, -0.3490658503988659), sp.Eq(Arm_b, 2.0943951023931953), sp.Eq(Arm_c, 0)),
      "A4": sp.And(sp.Eq(Arm_a, -0.3490658503988659), sp.Eq(Arm_b, 2.0943951023931953), sp.Eq(Arm_c, 0)),
      "A5": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, arm.minAngleAB), sp.Eq(Arm_c, 0)),
      "B0": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B1": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0.3), sp.Eq(Cart_y, 0)),
      "B2": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0.3), sp.Eq(Cart_y, 0)),
      "B3": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0.3), sp.Eq(Cart_y, 0)),
      "C0": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0), sp.Eq(Carrier_y, 0)),
      "C1": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0.5), sp.Eq(Carrier_y, 0)),
      "C2": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0.5), sp.Eq(Carrier_y, 0))
    }

