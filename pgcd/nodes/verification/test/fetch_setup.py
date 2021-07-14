import sympy as sp
from verification.spec.component import World
from mpmath import mp
from cart import *
from arm import *

# Fetch

def progFetchA():
    return '''
        while( true ) {
        A0: receive(C, m_Idle) {
                case fold() => {
        A1:         m_Fold;
                    send(C, folded, 0);
                }
                case grab(loc) => {
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
        C0: receive(A, m_Idle){
            case folded() => skip;
        }
        while (sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1) {
        C1:    m_MoveFromTo(Pnt(0,0,0), Pnt(2,0,0));
        }
        send(A, grab, Pnt(2.2,0,0));
        C2: receive(A, m_Idle){
            case grabbed() => skip;
        }
        send(A, fold, 0);
        C3: receive(A, m_Idle){
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
