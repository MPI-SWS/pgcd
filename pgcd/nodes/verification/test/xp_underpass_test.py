import spec.conf
from spec.component import Cube
from spec.contract import StaticContract
from spec.time import DurationSpec
from utils.geometry import *
from sympy import symbols, Eq, And, S
from cart import Cart
from cart import CartSquare
from arm import Arm
from mpmath import mp
from experiments_setups import World, XpTestHarness

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

#TODO Cart idle sync is funny, should be wait
def xp1_choreo():
    return ''' Bridge =
        def x0 = (Cart: Idle(), Arm: SetAngleTurntable(0, 1.5707963267948966, 10), Carrier: MoveCart(0, 0, 0, 0.67, 10) ) ; x1
            x1 = (Cart: Idle(), Arm: SetAngleCantilever(-2.2689280275926285, 1.3962634015954636), Carrier: Idle()) ; x2
            x2 = (Cart: Idle(), Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659), Carrier: Idle()) ; x3
            x3 = Carrier -> Arm: OK(); x4
            x4 = (Cart: Idle(), Arm: SetAngleCantilever(1.3962634015954636, 2.0943951023931953), Carrier: Idle()) ; x5
            x5 = (Cart: Idle(), Arm: Grip(9.5), Carrier: Idle()) ; x6
            x6 = (Cart: Idle(), Arm: SetAngleCantilever(2.0943951023931953, 0.87266462599716477), Carrier: Idle()) ; x7
            x7 = Arm -> Carrier: OK(); x8
            x8 = (Cart: Idle(), Arm: SetAngleTurntable(1.5707963267948966, 0, 5), Carrier: SetAngleCart(0.78539816339744828, 5)) ; x9
            x9 = (Cart: Idle(), Arm: Idle(), Carrier: MoveCart(0.67, 0, rad(45), 0.5)) ; x10
            x10 = (Cart: Idle(), Arm: Idle(), Carrier: SetAngleCart(rad(90))) ; x11
            x11 = (Cart: Idle(), Arm: Idle(), Carrier: StrafeCart(1.023553390594, 0.353553390594, rad(90), 0.085)) ; x12
            x12 = Carrier -> Arm: OK(); x13
            x13 = (Cart: Idle(), Arm: SetAngleCantilever(0.87266462599716477, 2.0943951023931953), Carrier: Idle()) ; x14
            x14 = (Cart: Idle(), Arm: Grip(6), Carrier: Idle()) ; x15
            x15 = Arm -> Carrier: OK(); x16
            x16 = (Cart: Idle(), Arm: RetractArm(0, 2.0943951023931953, -0.3490658503988659, 10), Carrier: MoveCart(1.108553390594, 0.353553390594, rad(90), 0.7, 10)) ; x17
            x17 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

#this needs contract for the mounting point of the arm

class CartContract(StaticContract):

    def __init__(self, component, fp, duration = DurationSpec(1, 1, False)):
        ct, cx, cy = symbols("Cart_theta Cart_x Cart_y")
        g = And(Eq(cx, 0), Eq(cy, 0), Eq(ct, 0))
        super().__init__("CartContract", {component}, S.true, g, fp, duration)

class ArmCarrierContract(StaticContract):
    
    def __init__(self, c1, c2, fp, duration = DurationSpec(1, 1, False)):
        ct, cx, cy = symbols("Cart_theta Cart_x Cart_y")
        a = And(Eq(cx, 0), Eq(cy, 0), Eq(ct, 0))
        super().__init__("ArmCarrierContract", {c1, c2}, a, S.true, fp, duration)

class ArmContract(StaticContract):
    
    def __init__(self, component, fp, duration = DurationSpec(1, 1, False)):
        ct, cx, cy = symbols("Cart_theta Cart_x Cart_y")
        a = And(Eq(cx, 0), Eq(cy, 0), Eq(ct, 0))
        super().__init__("ArmContract", {component}, a, S.true, fp, duration)


def xp2_choreo():
    return ''' Bridge =
        def x0 = @CartContract(Cart,               fpz < 0.09 && fpx > -0.25 && fpy < 0.26 && fpx - fpy > -0.29 ) idler0 ||
                 @ArmCarrierContract(Arm, Carrier, fpz > 0.1  || fpx < -0.25 || fpy > 0.26 || fpx - fpy < -0.30 ) x1
            x1 = @ArmContract(Arm, (fpz > 0.11 && fpx > -0.26) ||
                                   (fpz > 0.18 && fpx < -0.25) ) a0 ||
                 { fpz < 0.17 && fpx < -0.27 } c0
            idler0 = (Cart: Idle()); idler1
            a0 = (Arm: SetAngleTurntable(0, 1.5707963267948966, 3)); a1
            a1 = (Arm: SetAngleCantilever(-2.2689280275926285, 1.3962634015954636, 3)); a2
            a2 = (Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659, 3)) ; a3
            a3 = (Arm: Idle()) ; a33
            c0 = (Carrier: MoveCart(0, 0, 0, 0.67, 10) ); c1
            a33 || c1 = x3
            x3 = Carrier -> Arm: OK(); x4
            x4 = @ArmContract(Arm, (fpz > 0.11 && fpx - fpy > -0.37 && fpx > -0.26) ||
                                   (fpz > 0.18 && fpx < -0.25) ) a4 ||
                 { fpz < 0.17 && fpx < -0.27 } c4
            a4 = (Arm: SetAngleCantilever(1.3962634015954636, 2.0943951023931953)) ; a5
            a5 = (Arm: Grip(9.5)) ; a6
            a6 = (Arm: SetAngleCantilever(2.0943951023931953, 0.87266462599716477)) ; a7
            c4 = (Carrier: Idle()); c5
            a7 || c5 = x7
            x7 = Arm -> Carrier: OK(); x8
            x8 = @ArmContract(Arm, (fpz > 0.11 &&  fpx > -0.27 && fpy < 0.27 && fpx - fpy > -0.31) ||
                                   (fpz > 0.18 && (fpx < -0.26 || fpy > 0.26 || fpx - fpy < -0.30)) ) a8 ||
                 { fpz < 0.17 && (fpx < -0.27 || fpy > 0.28 || fpx - fpy < -0.32) } c8
            a8 = (Arm: SetAngleTurntable(1.5707963267948966, 0, 5)) ; a9
            a9 = (Arm: Idle()) ; a10
            c8 = (Carrier: SetAngleCart(0.78539816339744828, 5)) ; c9
            c9 = (Carrier: MoveCart(0.67, 0, rad(45), 0.5)) ; c10
            c10 = (Carrier: SetAngleCart(rad(90))) ; c11
            c11 = (Carrier: StrafeCart(1.023553390594, 0.353553390594, rad(90), 0.085)) ; c12
            a10 || c12 = x12
            x12 = Carrier -> Arm: OK(); x13
            x13 = @ArmContract(Arm, (fpz > 0.11 && fpy < 0.26) ||
                                    (fpz > 0.18 && fpy > 0.25) ) a13 ||
                  { fpz < 0.17 && fpy > 0.27 } c13
            a13 = (Arm: SetAngleCantilever(0.87266462599716477, 2.0943951023931953)) ; a14
            a14 = (Arm: Grip(6)) ; a15
            c13 = (Carrier: Idle()) ; c14
            a15 || c14 = x15
            x15 = Arm -> Carrier: OK(); x16
            x16 = @ArmContract(Arm, (fpz > 0.11 && fpy < 0.26) ||
                                    (fpz > 0.18 && fpy > 0.25) ) a16 ||
                  { fpz < 0.17 && fpy > 0.27 } c16
            a16 = (Arm: RetractArm(0, 2.0943951023931953, -0.3490658503988659, 10)) ; a17
            c16 = (Carrier: MoveCart(1.108553390594, 0.353553390594, rad(90), 0.7, 10)) ; c17
            a17 || c17 = x17
            idler1 || x17 = x18
            x18 = Arm-> Cart: OK(); x19
            x19 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

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

class XpUnderpass(XpTestHarness):
    
    def test_underpass(self):
        ch = xp1_choreo()
        w = xp1_world() 
        contracts = []
        progs = { "Arm": xp1_arm(),
                  "Cart": xp1_cart(),
                  "Carrier": xp1_carrier() }
        self.check(ch, w, contracts, progs)

    def test_underpass_new(self):
        ch = xp2_choreo()
        w = xp1_world() 
        contracts = [CartContract, ArmCarrierContract, ArmContract]
        progs = { "Arm": xp2_arm(),
                  "Cart": xp2_cart(),
                  "Carrier": xp2_carrier() }
        self.check(ch, w, contracts, progs)

if __name__ == '__main__':
    unittest.main()
