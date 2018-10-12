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


#TODO twist and turn
