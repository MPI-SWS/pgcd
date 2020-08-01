import sympy as sp
from experiments_setups import World
from mpmath import mp
from cart import *
from arm import *

#> 180    3.1415926535897931
#  130    2.2689280275926285
#> 120    2.0943951023931953
#>  90    1.5707963267948966
#>  45    0.78539816339744828

def xp1_world():
    w       = World( (0,0,0, mp.pi/2),   # cart: it is rotated by 90 degrees, so it x/y are not the same as the global frame
                     (-1.1, 0.09, 0, 0), # carrier: aligned with the right side
                     (0,0,0, 0),         # pillar
                     (0,0,0, 0) )        # bridge
    cart    = Cart("Cart", w, 0)
    arm     = Arm("Arm", cart)           # on top the cart: it is also rotated by 90 degrees
    carrier = CartSquare("Carrier", w, 1)
    pillar  = Cube( -0.5, 0.5, 0, mp.pi/4, 0.1, 0.3, 0.2, w, 2)
    bridge  = Cube( -0.5, 0.5, 0.2, mp.pi/4, 0.3, 0.03, 0.1, w, 3)
    return w

def xp1_cart():
    return '''
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    idle;
    '''

def xp2_cart():
    return '''
    receive(Arm, Idle ){
        case OK() => skip;
    }
    '''

def xp1_arm():
    return '''
    # 1, Wait for carrier
    setAngleTurntable( 90 );
    setAngleCantilever(210);
    setAngleAnchorPoint( 150 );

    #grab and lift piece
    receive(Carrier, Idle ){
        case OK() => skip;
    }
    setAngleCantilever(250);
    grip(9.5);
    setAngleCantilever( 180 );

    # 2, release carrier and move to second configuration
    send( Carrier, OK, 1.0 );

    setAngleTurntable( 0 );

    # 3, wait for the carrier to arrive, release piece
    receive(Carrier, Idle ){
        case OK() => skip;
    }
    setAngleCantilever( 250 );
    grip( 6 );

    # 4, release carrier, move to final configuration
    send( Carrier, OK, 1.0 );
    retractArm;
    '''

def xp2_arm():
    return '''
    # 1, Wait for carrier
    setAngleTurntable( 90 );
    setAngleCantilever(210);
    setAngleAnchorPoint( 150 );

    #grab and lift piece
    receive(Carrier, Idle ){
        case OK() => skip;
    }
    setAngleCantilever(250);
    grip(9.5);
    setAngleCantilever( 180 );

    # 2, release carrier and move to second configuration
    send( Carrier, OK, 1.0 );

    setAngleTurntable( 0 );

    # 3, wait for the carrier to arrive, release piece
    receive(Carrier, Idle ){
        case OK() => skip;
    }
    setAngleCantilever( 250 );
    grip( 6 );

    # 4, release carrier, move to final configuration
    send( Carrier, OK, 1.0 );
    retractArm;
    send( Cart, OK, 1.0 );
    '''

def xp1_carrier():
    return '''
    #Going for arm
    moveCart( 670 );
    Idle;
    Idle;

    # 1, Send in position to arm
    send( Arm, OK, 1.0 );
    # 2, Get release from arm and move to second position
    receive(Arm, Idle ){
        case OK() => skip;
    }
    setAngleCart( 45 );
    moveCart( 500 );
    setAngleCart( 85 );
    strafeCart( 85 );

    # 3, Send in position to arm
    send( Arm, OK, 1.0 );
    # 4, Get release from arm and move to final position
    receive(Arm, Idle ){
        case OK() => skip;
    }

    moveCart( 700 );
    '''

def xp2_carrier():
    return '''
    #Going for arm
    moveCart( 670 );

    # 1, Send in position to arm
    send( Arm, OK, 1.0 );

    # 2, Get release from arm and move to second position
    receive(Arm, Idle ){
        case OK() => skip;
    }
    setAngleCart( 45 );
    moveCart( 500 );
    setAngleCart( 85 );
    strafeCart( 85 );

    # 3, Send in position to arm
    send( Arm, OK, 1.0 );
    # 4, Get release from arm and move to final position
    receive(Arm, Idle ){
        case OK() => skip;
    }

    moveCart( 700 );
    '''
def progUnderpassArm():
    return '''
    A0: SetAngleTurntable(0, 1.5707963267948966);
    A1: SetAngleCantilever(-2.2689280275926285, 1.3962634015954636);
    A2: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659);
    A3: receive(Carrier, Idle ){
        case OK() => skip;
    }
    A4: SetAngleCantilever(1.3962634015954636, 2.0943951023931953);
    A5: Grip(9.5);
    A6: SetAngleCantilever(2.0943951023931953, 0.87266462599716477);
    send( Carrier, OK, 1.0 );
    A7: SetAngleTurntable(1.5707963267948966, 0);
    A8: receive(Carrier, Idle ){
        case OK() => skip;
    }
    A9: SetAngleCantilever(0.87266462599716477, 2.0943951023931953 );
    A10: Grip( 6 );
    send( Carrier, OK, 1.0 );
    A11: RetractArm;
    '''

def progUnderpassCart():
    return '''
    B0: Idle;
    B1: Idle;
    B2: Idle;
    B3: Idle;
    B4: Idle;
    B5: Idle;
    B6: Idle;
    B7: Idle;
    B8: Idle;
    B9: Idle;
    B10: Idle;
    B11: Idle;
    B12: Idle;
    '''

def progUnderpassCarrier():
    return '''
    C0: MoveCart(0, 0, 0, 0.67);
    C1: Idle;
    C2: Idle;
    send( Arm, OK, 1.0 );
    C3: receive(Arm, Idle ){
        case OK() => skip;
    }
    C4: SetAngleCart(0.78539816339744828);
    C5: MoveCart(0.67, 0, 0.78539816339744828, 0.5);
    C6: SetAngleCart(1.5707963267948966);
    C7: StrafeCart(1.023553390594, 0.353553390594, 1.5707963267948966, -0.085);
    send( Arm, OK, 1.0 );
    C8: receive(Arm, Idle ){
        case OK() => skip;
    }
    C9: MoveCart(1.108553390594, 0.353553390594, 1.5707963267948966, 0.7);
    '''

def progUnderpassAnnot(arm, car, carrier):
    Arm_a, Arm_b, Arm_c = sp.symbols('Arm_a, Arm_b, Arm_c')
    Cart_x, Cart_y, Cart_theta = sp.symbols('Cart_x, Cart_y, Cart_theta')
    Carrier_x, Carrier_y, Carrier_theta = sp.symbols('Carrier_x, Carrier_y, Carrier_theta')
    return {
      "A0": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, arm.minAngleAB), sp.Eq(Arm_c, 0)),
      "A1": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, arm.minAngleAB), sp.Eq(Arm_c, 1.5707963267948966)),
      "A2": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 1.3962634015954636), sp.Eq(Arm_c, 1.5707963267948966)),
      "A3": sp.And(sp.Eq(Arm_a, -0.3490658503988659), sp.Eq(Arm_b, 1.3962634015954636), sp.Eq(Arm_c, 1.5707963267948966)),
      "A4": sp.And(sp.Eq(Arm_a, -0.3490658503988659), sp.Eq(Arm_b, 1.3962634015954636), sp.Eq(Arm_c, 1.5707963267948966)),
      "A5": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 2.0943951023931953), sp.Eq(Arm_c, 1.5707963267948966)),
      "A6": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 2.0943951023931953), sp.Eq(Arm_c, 1.5707963267948966)),
      "A7": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 0.87266462599716477), sp.Eq(Arm_c, 1.5707963267948966)),
      "A8": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 0.87266462599716477), sp.Eq(Arm_c, 0)),
      "A9": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 0.87266462599716477), sp.Eq(Arm_c, 0)),
      "A10": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 2.0943951023931953), sp.Eq(Arm_c, 0)),
      "A11": sp.And(sp.Eq(Arm_a, arm.maxAngleAB), sp.Eq(Arm_b, 2.0943951023931953), sp.Eq(Arm_c, 0)),
      "B0": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B1": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B2": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B3": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B4": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B5": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B6": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B7": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B8": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B9": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B10": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B11": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "B12": sp.And(sp.Eq(Cart_theta, 0), sp.Eq(Cart_x, 0), sp.Eq(Cart_y, 0)),
      "C0": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0), sp.Eq(Carrier_y, 0)),
      "C1": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0.67), sp.Eq(Carrier_y, 0)),
      "C2": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0.67), sp.Eq(Carrier_y, 0)),
      "C3": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0.67), sp.Eq(Carrier_y, 0)),
      "C4": sp.And(sp.Eq(Carrier_theta, 0), sp.Eq(Carrier_x, 0.67), sp.Eq(Carrier_y, 0)),
      "C5": sp.And(sp.Eq(Carrier_theta, 0.78539816339744828), sp.Eq(Carrier_x, 0.67), sp.Eq(Carrier_y, 0)),
      "C6": sp.And(sp.Eq(Carrier_theta, 0.78539816339744828), sp.Eq(Carrier_x, 1.023553390594), sp.Eq(Carrier_y, 0.353553390594)),
      "C7": sp.And(sp.Eq(Carrier_theta, 1.5707963267948966), sp.Eq(Carrier_x, 1.023553390594), sp.Eq(Carrier_y, 0.353553390594)),
      "C8": sp.And(sp.Eq(Carrier_theta, 1.5707963267948966), sp.Eq(Carrier_x, 1.108553390594), sp.Eq(Carrier_y, 0.353553390594)),
      "C9": sp.And(sp.Eq(Carrier_theta, 1.5707963267948966), sp.Eq(Carrier_x, 1.108553390594), sp.Eq(Carrier_y, 0.353553390594))
    }

