from spec.component import Cube
from compatibility import *
from utils.geometry import *
from cart import Cart
from cart import CartSquare
from arm import Arm
from refinement import *
from vectorize import *
from mpmath import mp
from experiments_setups import World
from copy import deepcopy
from choreography.projection import Projection
import interpreter.parser as parser
import time

import unittest

#> 180    3.1415926535897931
#  130    2.2689280275926285
#> 120    2.0943951023931953
#>  90    1.5707963267948966
#>  45    0.78539816339744828

def xp1_world():
    w       = World( (0,0,0, mp.pi/4), # cart
                     (-1.1, 0, 0, 0),  # carrier
                     (0,0,0, 0),       # pillar
                     (0,0,0, 0) )      # bridge 
    cart    = Cart("Cart", w, 0)
    arm     = Arm("Arm", cart)
    carrier = CartSquare("Carrier", w, 1)
    pillar  = Cube( -0.5, 0.5, 0, mp.pi/4, 0.1, 0.3, 0.2, w, 2)
    bridge  = Cube( -0.5, 0.5, 0.2, mp.pi/4, 0.3, 0.03, 0.1, w, 3)
    return w

# TODO collision check seems to be missing static objects !!
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
            x16 = (Cart: Idle(), Arm: RetractArm(10), Carrier: MoveCart(1.108553390594, 0.353553390594, rad(90), 0.7, 10)) ; x17
            x17 = end
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


class XpUnderpass(unittest.TestCase):

    def test_underpass(self, debug = False):
        w = xp1_world() 
        ch = xp1_choreo()
        progs = { "Arm": xp1_arm(),
                  "Cart": xp1_cart(),
                  "Carrier": xp1_carrier() }
        start = time.time()
        visitor = Projection()
        visitor.execute(ch, w, debug)
        chor = visitor.choreography
        vectorize(chor, w)
        end = time.time()
        print("Syntactic checks:", end - start)
        start = end
        checker = CompatibilityCheck(chor, w)
        checker.localChoiceChecks()
        checker.generateTotalGuardsChecks()
        checker.computePreds(debug)
        checker.generateCompatibilityChecks(debug)
        end = time.time()
        print("VC generation:", end - start)
        start = end
        print("#VC:", len(checker.vcs))
        for i in range(3, len(checker.vcs)): # XXX skip the one about the processes abstract FP
            vc = checker.vcs[i]
            print("Checking VC", i, vc.title)
            if not vc.discharge(debug=debug):
                raise Exception(str(vc))
        end = time.time()
        print("VC solving:", end - start)
        start = end
        processes = w.allProcesses()
        for p in processes:
            visitor.choreography = deepcopy(chor)
            proj = visitor.project(p.name(), p, debug)
            prser = parser.Parser()
            prog = prser.parse(progs[p.name()])
            ref = Refinement(prog, proj, debug)
            if not ref.check():
                raise Exception("Refinement: " + p.name())
        end = time.time()
        print("refinement:", end - start)
        start = end

if __name__ == '__main__':
    unittest.main()
