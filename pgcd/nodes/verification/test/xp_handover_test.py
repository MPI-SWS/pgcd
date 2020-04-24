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

def xp2_world():
    w = World(  (0,0,0,0),
                (1, 0, 0, mp.pi) )
    cart = Cart("Cart", w, 0)
    arm = Arm("Arm", cart)
    carrier = CartSquare("Carrier", w, 1)
    return w

def xp2_choreo_1():
    return ''' Handover =
        def x0 = (Cart: MoveCart(0, 0, 0, 0.3, 5), Arm: Idle(), Carrier: MoveCart(0, 0, 0, 0.5, 5) ) ; x1
            x1 = Carrier -> Cart: OK(); x2
            x2 = Cart -> Arm: OK(); x3
            x3 = (Cart: Idle(), Arm: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953), Carrier: Idle() ) ; x4
            x4 = (Cart: Idle(), Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659), Carrier: Idle() ) ; x5
            x5 = (Cart: Idle(), Arm: Grip(9.5), Carrier: Idle() ) ; x6
            x6 = (Cart: Idle(), Arm: RetractArm(), Carrier: Idle() ) ; x7
            x7 = Arm -> Cart: OK(); x8
            x8 = Cart -> Carrier: OK(); x9
            x9 = (Cart: MoveCart(0.3, 0, 0, -0.3, 5), Arm: Idle(), Carrier: MoveCart(0.5, 0, 0, -0.5, 5) ) ; x10
            x10 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

# TODO when we split the cart+arm and check the arm's FP it is underconstrained as we loose the contraints on the cart's mounting point!
# TODO need a way of (1) specifying frame for the FP spec in the annotations and (2) for the VC flatten to least common ancestor rather than world
def xp2_choreo_2():
    return ''' Handover =
        def x0 = { fpx < 0.4 } ca0 ||
                 { fpx > 0.4 } c0
            ca0 = (Cart: MoveCart(0, 0, 0, 0.3, 5), Arm: Idle()) ; ca1
            c0 = (Carrier: MoveCart(0, 0, 0, 0.5, 5) ) ; c1
            ca1 || c1 = x1
            x1 = Carrier -> Cart: OK(); x2
            x2 = Cart -> Arm: OK(); x3
            x3 = { (fpx > 0.4) && (fpz < 0.17) } cr3 ||
                 { (fpx < 0.4) || (fpz > 0.17) } ct3
            ct3 = (Cart: Idle(), Arm: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953)) ; ct4
            ct4 = (Cart: Idle(), Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659)) ; ct5
            ct5 = (Cart: Idle(), Arm: Grip(9.5) ) ; ct6
            ct6 = (Cart: Idle(), Arm: RetractArm()) ; ct7
            cr3 = (Carrier: Idle()) ; cr7
            ct7 || cr7 = x7
            x7 = Arm -> Cart: OK(); x8
            x8 = Cart -> Carrier: OK(); x9
            x9 = { fpx < 0.4 } ca9 ||
                 { fpx > 0.4 } c9
            ca9 = (Cart: MoveCart(0.3, 0, 0, -0.3, 5), Arm: Idle()) ; ca10
            c9 = (Carrier: MoveCart(0.5, 0, 0, -0.5, 5) ) ; c10
            ca10 || c10 = x10
            x10 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

def xp2_choreo_3():
    return ''' Handover =
        def x0 = { fpx < 0.4 } ca0 ||
                 { fpx > 0.4 } c0
            ca0 = (Cart: MoveCart(0, 0, 0, 0.3, 5), Arm: Idle()) ; ca1
            c0 = (Carrier: MoveCart(0, 0, 0, 0.5, 5) ) ; c1
            ca1 || c1 = x1
            x1 = Carrier -> Cart: OK(); x2
            x2 = Cart -> Arm: OK(); x3
            x3 = { (fpx > 0.4) && (fpz < 0.17) } cr3 ||
                 { (fpx < 0.4) && (fpz < 0.09) } ct3 ||
                 { (((fpx < 0.4) && (fpz > 0.09)) || (fpz > 0.17)) && (Cart_theta == 0) && (Cart_x == 0.3) && (Cart_y == 0) } ar3
            ar3 = (Arm: SetAngleCantilever(-2.2689280275926285, 2.0943951023931953)) ; ar4
            ar4 = (Arm: SetAngleAnchorPoint(2.2689280275926285, -0.3490658503988659)) ; ar5
            ar5 = (Arm: Grip(9.5) ) ; ar6
            ar6 = (Arm: RetractArm()) ; ar7
            ct3 = (Carrier: Idle()) ; ct7
            cr3 = (Carrier: Idle()) ; cr7
            ar7 || ct7 || cr7 = x7
            x7 = Arm -> Cart: OK(); x8
            x8 = Cart -> Carrier: OK(); x9
            x9 = { fpx < 0.4 } ca9 ||
                 { fpx > 0.4 } c9
            ca9 = (Cart: MoveCart(0.3, 0, 0, -0.3, 5), Arm: Idle()) ; ca10
            c9 = (Carrier: MoveCart(0.5, 0, 0, -0.5, 5) ) ; c10
            ca10 || c10 = x10
            x10 = end
        in [  (Cart_theta == 0) && (Cart_x == 0) && (Cart_y == 0) &&
              (Carrier_theta == 0) && (Carrier_x == 0) && (Carrier_y == 0) &&
              (Arm_a == 2.2689280275926285) && (Arm_b == -2.2689280275926285) && (Arm_c == 0) ] x0
    '''

def xp2_cart():
    return '''
    #1,  go to meeting position
    moveCart( -300 );
    receive(Carrier, Idle ){ 
        case OK() => skip;
    }
    send( Arm, OK, 1.0 );
    # 2, wait and return
    receive(Arn, Idle ){
        case OK() => skip;
    }
    send( Carrier, OK, 1.0 );
    moveCart( 300 );
    '''

def xp2_arm():
    return '''
    # 1, start to grab 
    receive(Cart, Idle ){
        case OK() => skip;
    }
    setAngleCantilever( 250 );
    setAngleAnchorPoint( 150 );
    grip( 9.5 );
    retractArm;
    # 2, signal cart and carrier to move
    send( Cart, OK, 1.0 );
    idle;
    '''

def xp2_carrier():
    return '''
    # 1, Going for arm, send message
    moveCart( 500 );
    send( Cart, OK, 1.0 );
    # 2, wait for arm to grab
    receive(Cart, Idle ){
        case OK() => skip;
    }
    # 3, return home
    moveCart( -500 );
    '''

class XpHandoverTest(unittest.TestCase):

    def handover(self, ch, debug = False):
        w = xp2_world()
        progs = { "Arm": xp2_arm(),
                  "Cart": xp2_cart(),
                  "Carrier": xp2_carrier() }
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
    
    def test_handover_1(self, debug = False):
        self.handover(xp2_choreo_1(), debug)

    def test_handover_2(self, debug = False):
        self.handover(xp2_choreo_2(), debug)

    def test_handover_3(self, debug = False):
        self.handover(xp2_choreo_3(), debug)

if __name__ == '__main__':
    unittest.main()
