from sympy import *
from sympy.vector import CoordSys3D
from spec import *
from geometry import *
from cart import *
from arm import *
from mpmath import mp

#>>> mpmath.pi()      mpf('3.1415926535897931')
#>>> mpmath.pi()/2    mpf('1.5707963267948966')
#>>> mpmath.pi()/4    mpf('0.78539816339744828')

def xp1_world():
    w = World( (0,0,0, m.pi/4), (-1, 0, 0, 0) )
    cart = Cart("Cart", w, 0)
    arm = Arm("Arm", cart)
    carrier = Cart("Carrier", w, 1)
    pillar = Cube( -0.4, 0.4, 0, m.pi/4, 0.1, 0.3, 0.2)
    bridge = Cube( -0.4, 0.4, 0.2, m.pi/4, 0.3, 0.03, 0.1)
    return w

def xp1_choreo():
    return ''' Bridge =
        def x0 = (Cart: Idle(), Arm: SetAngleTurntaburntable(), Carrier: MoveCart(0, 0, 0, 0.67) ) ; x1
            x1 = (Cart: Idle(), Arm: SetAngleCantilever(), Carrier: Idle()) ; x2
            x2 = (Cart: Idle(), Arm: SetAngleAnchorPoint(), Carrier: Idle()) ; x3
            x3 = Carrier -> Arm: OK(); x4
            x4 = (Cart: Idle(), Arm: SetAngleCantilever(), Carrier: Idle()) ; x5
            x5 = (Cart: Idle(), Arm: Grip(), Carrier: Idle()) ; x6
            x6 = (Cart: Idle(), Arm: SetAngleCantilever(), Carrier: Idle()) ; x7
            x7 = Arm -> Carrier: OK(); x8
            x8 = (Cart: Idle(), Arm: SetAngleTurntaburntable(), Carrier: SetAngleCart(0.78539816339744828)) ; x9
            x9 = (Cart: Idle(), Arm: Idle(), Carrier: MoveCart(0.67, 0, 0.78539816339744828, 0.45)) ; x10
            x10 = (Cart: Idle(), Arm: Idle(), Carrier: SetAngleCart(1.5707963267948966)) ; x11
            x11 = (Cart: Idle(), Arm: Idle(), Carrier: StrafeCart(x, y, 1.5707963267948966, 0.085)) ; x12
            x12 = Carrier -> Arm: OK(); x13
            x13 = (Cart: Idle(), Arm: SetAngleCantilever(), Carrier: Idle()) ; x14
            x14 = (Cart: Idle(), Arm: Grip(), Carrier: Idle()) ; x15
            x15 = (Cart: Idle(), Arm: SetAngleCantilever(), Carrier: Idle()) ; x16
            x16 = Arm -> Carrier: OK(); x17
            x17 = (Cart: Idle(), Arm: RetractArm(), Carrier: MoveCart(x, y, t, dist)) ; x18
            x18 = end
        in [ (Cart_x == 0) && (Cart_y == 0) && (Carrier_x == 0) && (Carrier_y == 0) && (Arm_a == 0) && (Arm_b == 0) && (Arm_c == 0) ] x0
    '''

def xp1_cart():
    return '''
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle;
    m_idle
    '''

def xp1_arm():
    return '''
    # 1, Wait for carrier
    m_setAngleTurntable( 90 );
    m_setAngleAnchorPoint( 150 );
    m_setAngleCantilever(210);

    #grab and lift piece
    receive( m_Idle ){ (msg_OK, ok, {skip}) };
    m_setAngleCantilever(250);
    m_grip(9.5);
    m_setAngleCantilever( 180 );

    # 2, release carrier and move to second configuration
    send( id_carrier, msg_OK, 1.0 );

    m_setAngleTurntable( 0 );

    # 3, wait for the carrier to arrive, release piece
    receive( m_Idle ){ (msg_OK, ok, {skip}) };
    m_setAngleCantilever( 250 );
    m_grip( 6 );

    # 4, release carrier, move to final configuration
    send( id_carrier, msg_OK, 1.0 );
    m_retractArm
    '''


def xp1_carrier():
    return '''
    #Going for arm
    m_moveCart( 670 );

    # 1, Send in position to arm 
    send( id_arm, msg_OK, 1.0 );

    # 2, Get release from arm and move to second position
    receive( m_Idle ){ (msg_OK, ok, {skip} ) };
    m_setAngleCart( 45 );
    m_moveCart( 500 );
    m_setAngleCart( 85 );
    m_strafeCart( 85 );

    # 3, Send in position to arm 
    send( id_arm, msg_OK, 1.0 );
    # 4, Get release from arm and move to final position
    receive( m_Idle ){ (msg_OK, ok, {skip} ) };

    m_moveCart( 700 )
    '''

def xp2_world():
    w = World( (0,0,0,0), (1, 0, 0, m.pi) )
    cart = Cart("Cart", w, 0)
    arm = Arm("Arm", cart)
    carrier = Cart("Carrier", w, 1)
    return w

def xp2_choreo():
    return ''' Handover =
        def x0 = (Cart: MoveCart(x,y,t,dist), Arm: Idle(), Carrier: MoveCart(x, y, t, dist) ) ; x1
            x1 = Carrier -> Cart: OK(); x2
            x2 = Cart -> Arm: OK(); x3
            x3 = (Cart: Idle(), Arm: SetAngleCantilever(), Carrier: Idle() ) ; x4
            x4 = (Cart: Idle(), Arm: SetAngleAnchorPoint(), Carrier: Idle() ) ; x5
            x5 = (Cart: Idle(), Arm: Grip(), Carrier: Idle() ) ; x6
            x6 = (Cart: Idle(), Arm: RetractArm(), Carrier: Idle() ) ; x7
            x7 = Arm -> Cart: OK(); x8
            x8 = Cart -> Carrier: OK(); x9
            x9 = (Cart: MoveCart(x,y,t,dist), Arm: Idle(), Carrier: MoveCart(x, y, t, dist) ) ; x10
            10 = end
        in [ (Cart_x == 0) && (Cart_y == 0) && (Carrier_x == 0) && (Carrier_y == 0) && (Arm_a == 0) && (Arm_b == 0) && (Arm_c == 0) ] x0
    '''

def xp2_cart():
    return '''
    #1,  go to meeting position
    #receive( m_Idle ){ (msg_OK, ok, { skip} ) };
    m_moveCart( -300 );
    receive( m_Idle ){ (msg_OK, ok, { skip } ) };
    send( id_arm, msg_OK, 1.0 );

    # 2, wait and return
    receive( m_Idle ){ (msg_OK, ok, { skip} ) };
    send( id_carrier, msg_OK, 1.0 );

    m_moveCart( 300 )
    '''

def xp2_arm():
    return '''
    # 1, start to grab 
    receive( m_Idle ){ (msg_OK, ok, {skip} ) };
    m_setAngleCantilever( 250 );
    m_setAngleAnchorPoint( 150 );
    m_grip( 9.5 );
    m_retractArm;

    # 2, signal cart and carrier to move
    send( id_cart, msg_OK, 1.0 )
    '''

def xp2_carrier():
    return '''
    # 1, Going for arm, send message
    #send( id_arm, msg_OK, 1.0 );
    m_moveCart( 500 );
    send( id_cart, msg_OK, 1.0 );

    # 2, wait for arm to grab
    receive( m_Idle ){ (msg_OK, ok, {skip} ) };

    # 3, return home
    m_moveCart( -500 )
    '''
