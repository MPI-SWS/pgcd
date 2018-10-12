import sympy as sp

# Fetch

def progFetchA():
    return '''
        while( true ) {
        A0: receive(m_Idle) {
                case fold() => {
        A1:         m_Fold;
                    send(C, folded, 0);
                }
                case grab(loc) => {
        #A2:         m_Grab(loc);
        A2:         m_Grab(Pnt(2.2,0,0));
                    send(C, grabbed, 0);
                }
                case done() =>
                    exit(0);
            }
        }
    '''

def progFetchC():
    return '''
        send(A, fold, 0);
        C0: receive(m_Idle){
            case folded() => skip;
        }
        while (sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1) {
        C1:    m_MoveFromTo(Pnt(0,0,0), Pnt(2,0,0));
        }
        send(A, grab, Pnt(2.2,0,0));
        C2: receive(m_Idle){
            case grabbed() => skip;
        }
        send(A, fold, 0);
        C3: receive(m_Idle){
            case folded() => skip;
        }
        while (sqrt((C_x)**2 + (C_y - 0)**2) > 0.1) {
        C4:   m_MoveFromTo(Pnt(2,0,0), Pnt(0,0,0));
        }
        send(A, done, 0);
    '''


def progFecthAnnot(arm, cart):
    # The variables:
    A_a, A_b, A_c = sp.symbols('A_a, A_b, A_c')
    C_x, C_y, C_theta = sp.symbols('C_x, C_y, C_theta')
    return {
        "A0": sp.true,
        "A1": sp.true,
        "A2": sp.true,
        "C0": sp.And(sp.Eq(C_x, 0), sp.Eq(C_y, 0), sp.Eq(C_theta, 0)),
        "C1": sp.And(sp.Eq(A_a, arm.maxAngleAB), sp.Eq(A_b, arm.minAngleAB), sp.Eq(A_c, 0), C_x >= 0, C_x <= 2, sp.Eq(C_y, 0)),
        "C2": sp.And(sp.Eq(A_a, arm.maxAngleAB), sp.Eq(A_b, arm.minAngleAB), sp.Eq(A_c, 0), sp.Eq(C_x, 2), sp.Eq(C_y, 0)),
        "C3": sp.And(sp.Eq(C_x, 2), sp.Eq(C_y, 0)),
        "C4": sp.And(sp.Eq(A_a, arm.maxAngleAB), sp.Eq(A_b, arm.minAngleAB), sp.Eq(A_c, 0), C_x >= 0, C_x <= 2, sp.Eq(C_y, 0))
    }

# Handover

def progHandoverArm():
    return '''
    # 1, start to grab 
    A0: receive( Idle ){
        case OK() => skip;
    }
    #setAngleCantilever( 250 );
    A1: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953);
    #setAngleAnchorPoint( 150 );
    A2: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659);
    A3: Grip( 9.5 );
    A4: RetractArm;
    # 2, signal cart and carrier to move
    send( Cart, OK, 1.0 );
    A5: Idle;
    '''

def progHandoverCart():
    return '''
    #1,  go to meeting position
    B0: MoveCart(0, 0, 0, 0.3);
    B1: receive( Idle ){ 
        case OK() => skip;
    }
    send( Arm, OK, 1.0 );
    # 2, wait and return
    B2: receive( Idle ){
        case OK() => skip;
    }
    send( Carrier, OK, 1.0 );
    B3: MoveCart(0.3, 0, 0, -0.3);
    '''

def progHandoverCarrier():
    return '''
    # 1, Going for arm, send message
    C0: MoveCart(0, 0, 0, 0.5);
    send( Cart, OK, 1.0 );
    # 2, wait for arm to grab
    C1: receive( Idle ){
        case OK() => skip;
    }
    # 3, return home
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

#TODO Underpass

def progUnderpassArm():
    return '''
    # 1, Wait for carrier
    A0: SetAngleTurntable(0, 1.5707963267948966);
    A1: SetAngleCantilever(-2.2689280275926285, 1.3962634015954636);
    A2: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659);
    #grab and lift piece
    A3: receive( Idle ){
        case OK() => skip;
    }
    A4: SetAngleCantilever(1.3962634015954636, 2.0943951023931953);
    A5: Grip(9.5);
    A6: SetAngleCantilever(2.0943951023931953, 0.87266462599716477);
    # 2, release carrier and move to second configuration
    send( Carrier, OK, 1.0 );
    A7: SetAngleTurntable(1.5707963267948966, 0);
    # 3, wait for the carrier to arrive, release piece
    A8: receive( Idle ){
        case OK() => skip;
    }
    A9: SetAngleCantilever(0.87266462599716477, 2.0943951023931953 );
    A10: Grip( 6 );
    # 4, release carrier, move to final configuration
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
    #Going for arm
    C0: MoveCart(0, 0, 0, 0.67);
    C1: Idle;
    C2: Idle;
    # 1, Send in position to arm 
    send( Arm, OK, 1.0 );
    # 2, Get release from arm and move to second position
    C3: receive( Idle ){
        case OK() => skip;
    }
    C4: SetAngleCart(0.78539816339744828);
    C5: MoveCart(0.67, 0, 0.78539816339744828, 0.5);
    C6: SetAngleCart(1.5707963267948966);
    C7: StrafeCart(1.023553390594, 0.353553390594, 1.5707963267948966, -0.085);
    # 3, Send in position to arm 
    send( Arm, OK, 1.0 );
    # 4, Get release from arm and move to final position
    C8: receive( Idle ){
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

#TODO twist and turn
