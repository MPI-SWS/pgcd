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

def xp2_world():
    w = World( (0,0,0,0), (1, 0, 0, mp.pi) )
    cart = Cart("Cart", w, 0)
    arm = Arm("Arm", cart)
    carrier = Cart("Carrier", w, 1)
    return w

def xp2_choreo():
    return ''' Handover =
        def x0 = (Cart: MoveCart(0, 0, 0, 0.3), Arm: Idle(), Carrier: MoveCart(0, 0, 0, 0.5) ) ; x1
            x1 = Carrier -> Cart: OK(); x2
            x2 = Cart -> Arm: OK(); x3
            x3 = (Cart: Idle(), Arm: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953), Carrier: Idle() ) ; x4
            x4 = (Cart: Idle(), Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659), Carrier: Idle() ) ; x5
            x5 = (Cart: Idle(), Arm: Grip(9.5), Carrier: Idle() ) ; x6
            x6 = (Cart: Idle(), Arm: RetractArm(), Carrier: Idle() ) ; x7
            x7 = Arm -> Cart: OK(); x8
            x8 = Cart -> Carrier: OK(); x9
            x9 = (Cart: MoveCart(0.3, 0, 0, -0.3), Arm: Idle(), Carrier: MoveCart(0.5, 0, 0, -0.5) ) ; x10
            x10 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

def xp2_cart():
    return '''
    #1,  go to meeting position
    m_moveCart( -300 );
    receive( m_Idle ){ 
        case OK() => skip;
    }
    send( Arm, OK, 1.0 );
    # 2, wait and return
    receive( m_Idle ){
        case OK() => skip;
    }
    send( Carrier, OK, 1.0 );
    m_moveCart( 300 );
    '''

def xp2_arm():
    return '''
    # 1, start to grab 
    receive( m_Idle ){
        case OK() => skip;
    }
    m_setAngleCantilever( 250 );
    m_setAngleAnchorPoint( 150 );
    m_grip( 9.5 );
    m_retractArm;
    # 2, signal cart and carrier to move
    send( Cart, OK, 1.0 );
    m_idle;
    '''

def xp2_carrier():
    return '''
    # 1, Going for arm, send message
    m_moveCart( 500 );
    send( Cart, OK, 1.0 );
    # 2, wait for arm to grab
    receive( m_Idle ){
        case OK() => skip;
    }
    # 3, return home
    m_moveCart( -500 );
    '''

def run(ch, components, progs, debug = False):
        visitor = exec.ChoreographyExecutor()
        visitor.execute(ch)
        chor = visitor.choreography
        vectorize(chor, components)
        checker = CompatibilityCheck(chor, components)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        processes = components.allProcesses()
        checker.computePreds(debug)
        checker.generateCompatibilityChecks()
        for vc in checker.vcs:
            if not vc.discharge(debug=debug):
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

class XpHandoverTest(unittest.TestCase):

    def test_handover(self):
        xp2_progs = { "Arm": xp2_arm(), "Cart": xp2_cart(), "Carrier": xp2_carrier() }
        self.assertTrue(run(xp2_choreo(), xp2_world(), xp2_progs))

if __name__ == '__main__':
    unittest.main()
