from spec import *
from propagate_preds_chor import *
from geometry import *
from cart import *
from arm import *
from refinement import *
from vectorize_spec import *
from mpmath import mp
from experiments_setups import World
from copy import deepcopy
import choreography.executor_chor as exec
import parser

import unittest

#> 180    3.1415926535897931
#  130    2.2689280275926285
#> 120    2.0943951023931953
#>  90    1.5707963267948966
#>  45    0.78539816339744828

def xp1_world():
    w = World( (0,0,0, mp.pi/4), (-1.1, 0, 0, 0) )
    cart = Cart("Cart", w, 0)
    arm = Arm("Arm", cart)
    carrier = Cart("Carrier", w, 1)
    pillar = Cube( -0.5, 0.5, 0, mp.pi/4, 0.1, 0.3, 0.2)
    bridge = Cube( -0.5, 0.5, 0.2, mp.pi/4, 0.3, 0.03, 0.1)
    return w

def xp1_choreo():
    return ''' Bridge =
        def x0 = (Cart: Idle(), Arm: SetAngleTurntable(0, 1.5707963267948966), Carrier: MoveCart(0, 0, 0, 0.67) ) ; x1
            x1 = (Cart: Idle(), Arm: SetAngleCantilever(-2.2689280275926285, 1.3962634015954636), Carrier: Idle()) ; x2
            x2 = (Cart: Idle(), Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659), Carrier: Idle()) ; x3
            x3 = Carrier -> Arm: OK(); x4
            x4 = (Cart: Idle(), Arm: SetAngleCantilever(1.3962634015954636, 2.0943951023931953), Carrier: Idle()) ; x5
            x5 = (Cart: Idle(), Arm: Grip(9.5), Carrier: Idle()) ; x6
            x6 = (Cart: Idle(), Arm: SetAngleCantilever(2.0943951023931953, 0.87266462599716477), Carrier: Idle()) ; x7
            x7 = Arm -> Carrier: OK(); x8
            x8 = (Cart: Idle(), Arm: SetAngleTurntable(1.5707963267948966, 0), Carrier: SetAngleCart(0.78539816339744828)) ; x9
            x9 = (Cart: Idle(), Arm: Idle(), Carrier: MoveCart(0.67, 0, 0.78539816339744828, 0.5)) ; x10
            x10 = (Cart: Idle(), Arm: Idle(), Carrier: SetAngleCart(1.5707963267948966)) ; x11
            x11 = (Cart: Idle(), Arm: Idle(), Carrier: StrafeCart(1.023553390594, 0.353553390594, 1.5707963267948966, -0.085)) ; x12
            x12 = Carrier -> Arm: OK(); x13
            x13 = (Cart: Idle(), Arm: SetAngleCantilever(0.87266462599716477, 2.0943951023931953), Carrier: Idle()) ; x14
            x14 = (Cart: Idle(), Arm: Grip(6), Carrier: Idle()) ; x15
            x15 = Arm -> Carrier: OK(); x16
            x16 = (Cart: Idle(), Arm: RetractArm(), Carrier: MoveCart(1.108553390594, 0.353553390594, 1.5707963267948966, 0.7)) ; x17
            x17 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
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
    '''

def xp1_arm():
    return '''
    # 1, Wait for carrier
    m_setAngleTurntable( 90 );
    m_setAngleCantilever(210);
    m_setAngleAnchorPoint( 150 );

    #grab and lift piece
    receive( m_Idle ){
        case OK() => skip;
    }
    m_setAngleCantilever(250);
    m_grip(9.5);
    m_setAngleCantilever( 180 );

    # 2, release carrier and move to second configuration
    send( Carrier, OK, 1.0 );

    m_setAngleTurntable( 0 );

    # 3, wait for the carrier to arrive, release piece
    receive( m_Idle ){
        case OK() => skip;
    }
    m_setAngleCantilever( 250 );
    m_grip( 6 );

    # 4, release carrier, move to final configuration
    send( Carrier, OK, 1.0 );
    m_retractArm;
    '''

def xp1_carrier():
    return '''
    #Going for arm
    m_moveCart( 670 );
    m_Idle;
    m_Idle;

    # 1, Send in position to arm 
    send( Arm, OK, 1.0 );

    # 2, Get release from arm and move to second position
    receive( m_Idle ){
        case OK() => skip;
    }
    m_setAngleCart( 45 );
    m_moveCart( 500 );
    m_setAngleCart( 85 );
    m_strafeCart( 85 );

    # 3, Send in position to arm 
    send( Arm, OK, 1.0 );
    # 4, Get release from arm and move to final position
    receive( m_Idle ){
        case OK() => skip;
    }

    m_moveCart( 700 );
    '''


def run(ch, components, progs, debug = False):
        visitor = exec.ChoreographyExecutor()
        visitor.execute(ch)
        chor = visitor.choreography
        vectorize(chor, components)
        checker = CompatibilityCheck(chor.mk_state_to_node(), chor, components)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        processes = components.allProcesses()
        checker.computePreds(debug)
        checker.generateCompatibilityChecks()
        for vc in checker.vcs:
            if not vc.discharge(debug=False):
                raise Exception(str(vc))
        print("#VC:", len(checker.vcs))
        for p in processes:
            visitor.choreography = deepcopy(chor)
            proj = visitor.project(p.name(), p, debug)
            prser = parser.Parser()
            prog = prser.parse(progs[p.name()])
            ref = Refinement(prog, proj, debug)
            if not ref.check():
                raise Exception("Refinement:" + p.name())
        return True

class XpUnderpass(unittest.TestCase):

    def test_underpass(self):
        xp1_progs = { "Arm": xp1_arm(), "Cart": xp1_cart(), "Carrier": xp1_carrier() }
        self.assertTrue(run(xp1_choreo(), xp1_world(), xp1_progs, True))

if __name__ == '__main__':
    unittest.main()
