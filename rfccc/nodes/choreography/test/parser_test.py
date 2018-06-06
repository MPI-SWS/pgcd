import sys

import parser as rp
from choreography.projection import Projection
import unittest
from experiments_setups import *
from vectorize_chor import *
from copy import deepcopy


# The example from the paper
# origin is (0,0), target is (2,0)
def cartAndArmFetch():
    return ''' Fetch =
        def x0 = C -> A : fold() ; x1
            x1 = (C : Idle(), A : Fold()) ; x2
            x2 = A -> C : folded() ; x3
            x3 + x6 = x4
            x4 = [sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1] x5 + [sqrt((C_x - 2)**2 + (C_y - 0)**2) <= 0.1] x7
            x5 = (C : MoveFromTo(Pnt(0,0,0), Pnt(2,0,0)), A : Idle()) ; x6
            x7 = C -> A : grab(Pnt(2.2,0,0)) ; x8
            x8 = ( C : Idle(), A : Grab(Pnt(2.2,0,0))) ; x9
            x9 = A -> C : grabbed() ; x10
            x10 = C -> A : fold() ; x11
            x11 = (C : Idle(), A : Fold()) ; x12
            x12 = A -> C : folded() ; x13
            x13 + x16 = x14
            x14 = [sqrt((C_x - 0)**2 + (C_y - 0)**2) > 0.1] x15 + [sqrt((C_x - 0)**2 + (C_y - 0)**2) <= 0.1] x17
            x15 = (C : MoveFromTo(Pnt(2, 0, 0), Pnt(0, 0, 0)), A : Idle()) ; x16
            x17 = C -> A : done() ; x18
            x18 = end
        in [ (C_x == 0) && (C_y == 0) ]x0
    '''


# 2 arms next to each other handing an object to each other
# `loc` needs to be within the reach of both arms
# `delta` is a positive offset which is roughly the reach/length of the gripper also is has to be aligned with the position of A and B
def armsHandover():
    return ''' Handover =
        def x0 = A1 -> A2 : meetAt(Pnt(0,0,0.1)); x1
            x1 = (A1: MoveTo(Pnt(-0.05,0,0.1)), A2: MoveTo(Pnt(0.05,0,0.1))); x2
            x2 = A1 -> A2 : holding(); x3
            x3 = (A1: Idle(), A2: CloseGripper()); x4
            x4 = A2 -> A1 : holding(); x5
            x5 = (A1: OpenGripper(), A2: Idle()); x6
            x6 = A1 -> A2 : action(released); x7
            x7 = (A1: Fold(), A2: Fold()); x8
            x8 = end
        in [ (A1_a == 1.57079632679490) && (A1_b == 1.57079632679490) && (A1_c == 0) && (A2_a == 1.57079632679490) && (A2_b == 1.57079632679490) && (A2_c == 0) ] x0
    '''

def armsHandover1():
    return ''' Handover =
        def x0 = A -> B : meetAt(loc); x1
            x1 = (A: MoveTo(loc - delta), B: MoveTo(loc + delta)); x2
            x2 = A -> B : holding(); x3
            x3 = (A: Idle(), B: CloseGripper()); x4
            x4 = B -> A : holding(); x5
            x5 = (A: OpenGripper(), B: Idle()); x6
            x6 = A -> B : action(released); x7
            x7 = (A: MoveToOrigin(), B: MoveToOrigin()); x8
            x8 = end
        in [ (A_a == 1.5707963267949) && (A_b == 1.5707963267949) && (A_c == 0) && (B_a == 1.5707963267949) && (B_b == 1.5707963267949) && (B_c == 0) ] x0
    '''

# A sightly more complicated example with two arm sitting on one table with two bins between them.
# They need to put objects in the bin without crossing.
# Here is the picture:
# ┌─┐   ┌─┐
# │ │   │1│
# └─┘   └─┘
#   A         B
#       ┌─┐   ┌─┐
#       │2│   │ │
#       └─┘   └─┘
# A, B are the arms
# 1, 2 are the target bin
# 1 is located at Pnt(0.4,0.2,0)
# 2 is located at (0.4,-0.2,0)
# The unlabeled boxes next to the arm is where they get their respective objects from.
def binSorting():
    return ''' BinSorting =
        def x0 = (A: Idle(), B: Idle()); x1         # because we cannot loop to x0 (is that restriction really needed?)
            x1 + x31 + x32 + x33 + x34 + x35 = x2

            # B makes a choice whether is it going to put an object in a bin (B_dummy is a dummy variable to track the choice back to B)
            x2 = [B_dummy >= 0] x3 + [B_dummy <= 1]x4 + [B_dummy >= 0]x5
            x3 = B -> A : none(); x6
            x4 = B -> A : useBin1(); x7
            x5 = B -> A : useBin2(); x8

            # choice at A
            x6 = [A_dummy >= 0]x10 + [A_dummy <= 1]x11 + [A_dummy >= 0]x12
            x7 = [A_dummy >= 0]x13 + [A_dummy <= 1]x14 + [A_dummy >= 0]x15
            x8 = [A_dummy >= 0]x16 + [A_dummy <= 1]x17 + [A_dummy >= 0]x18

            x11 + x14 = _x11
            x12 + x15 + x18 = _x12

            x10 = A -> B : done(); x20
            _x11 = A -> B : wait(); x21
            _x12 = A -> B : wait(); x22
            x13 = A -> B : ok(); x23
            x16 = A -> B : ok(); x24
            x17 = A -> B : ok(); x25

            x20 = end
            x21 = (A: PutInBin(Pnt(0.3,0.2,0.2)), B: Idle()); x31
            x22 = (A: PutInBin(Pnt(0.3,-0.2,0.2)), B: Idle()); x32
            x23 = (A: Idle(), B: PutInBin(Pnt(0.3,0.2,0.2))); x33
            x24 = (A: Idle(), B: PutInBin(Pnt(0.3,-0.2,0.2))); x34
            x25 = (A: PutInBin(Pnt(0.3,0.2,0.2)), B: PutInBin(Pnt(0.3,-0.2,0.2))); x35

        in [(A_a == 1.5707963267949) && (A_b == 1.5707963267949) && (A_c == 0) && (B_a == 1.5707963267949) && (B_b == 1.5707963267949) && (B_c == 0)] x0
    '''


# 2 arms and 1 cart ferrying objects between them
# this example should be made a bit more realistic with:
# - the move result not exact (but cart still knows his position) so the cart tells the arm his position
# - the put in not exact as well so the arm tell the cart where it put the object
# - for the get, the cart tell the arm his position and where the other arm put the object
# - while the cart is moving the arms can do something else
# - while the cart is busy with A/B then B/A can do something else
def ferry():
    return ''' Ferry =
        def x0 = (A1: Idle(), A2: Idle(), C: Idle()); ca3
            ca3 = C -> A1 : rdy(); ca4
            ca4 = (A2: Idle(), A1: PutOnCart(), C: Idle()); ca5
            ca5 = A1 -> C : done(); ca2b
            ca2b = (A1: Idle(), A2: Idle(), C: MoveFromTo(Pnt(-1,0,0), Pnt(1,0,0))); cb3
            cb3 = C -> A2 : rdy(); cb4
            cb4 = (A1: Idle(), A2: GetFromCart(), C: Idle()); cb5
            cb5 = A2 -> C : done(); cb2a
            cb2a = (A1: Idle(), A2: Idle(), C: MoveFromTo(Pnt(1,0,0), Pnt(-1,0,0))); ca2b1
            ca2b1 = end
        in [ (A1_a == 1.57079632679490) && (A1_b == 1.57079632679490) && (A1_c == 0) &&
             (A2_a == 1.57079632679490) && (A2_b == 1.57079632679490) && (A2_c == 0) &&
             (C_x == 0) && (C_y == 0) ] x0
    '''

def ferry1():
    return ''' Ferry =
        def x0 = (A1: Idle(), A2: Idle(), C: Idle()); ca
            ca = ca1 || ca3
            ca1 = (A2: Idle()); ca2
            ca3 = C -> A1 : rdy(); ca4
            ca4 = (A1: PutOnCart(), C: Idle()); ca5
            ca5 = A1 -> C : done(); ca6
            ca2 || ca6 = ca2b
            ca2b = (A1: Idle(), A2: Idle(), C: MoveFromTo(Pnt(-1,0,0), Pnt(1,0,0))); cb
            cb = cb1 || cb3
            cb1 = (A1: Idle()); cb2
            cb3 = C -> A2 : rdy(); cb4
            cb4 = (A2: GetFromCart(), C: Idle()); cb5
            cb5 = A2 -> C : done(); cb6
            cb2 || cb6 = cb2a
            cb2a = (A1: Idle(), A2: Idle(), C: MoveFromTo(Pnt(1,0,0), Pnt(-1,0,0))); ca2b1
            ca2b1 = end
        in [true] x0
    '''

def funny_thread_partition():
    return ''' G =
        def x0 = [x] x1 + [x] x2
            x1 = x3 || x4
            x2 = x5 || x6
            x3 || x5 = x7
            x4 || x6 = x8
            x7 + x8 = x9
            x9 = end
        in [true] x0
    '''

def causal_ok():
    # needs processes C, A1, A2
    return ''' G =
        def x0 = C -> A1: msg(); x1
            x1 = A1 -> C: msg(); x2
            x2 = C -> A2: msg(); x3
            x3 = A2 -> C: msg(); x4
            x4 = end
        in [true] x0
    '''

def causal_err():
    # needs processes C, A1, A2
    return ''' G =
        def x0 = C -> A1: msg(); x1
            x1 = C -> A2: msg(); x2
            x2 = x3 || x4
            x3 = A1 -> C: msg(); x5
            x4 = A2 -> C: msg(); x6
            x5 || x6 = x7
            x7 = end
        in [true] x0
    '''

def thread_partition_err1():
    # needs processes C, A1, A2
    return ''' G =
        def x0 = C -> A1: msg(); x1
            x1 = C -> A2: msg(); x2
            x2 = x3 || x4
            x3 = (C: Idle(), A1: Idle()); x5
            x4 = (A1: Idle(), A2: Idle()); x6
            x5 || x6 = x7
            x7 = end
        in [true] x0
    '''

def thread_partition_err2():
    # needs processes C, A1, A2
    return ''' G =
        def x0 = C -> A1: msg(); x1
            x1 = C -> A2: msg(); x2
            x2 = x3 || x4
            x3 = (C: Idle()); x5
            x4 = (A2: Idle()); x6
            x5 || x6 = x7
            x7 = end
        in [true] x0
    '''

def thread_partition_err3():
    # needs processes C, A1, A2
    return ''' G =
        def x0 = C -> A1: msg(); x1
            x1 = C -> A2: msg(); x2
            x2 = x3 || x4
            x3 = (C: Idle(), A1: Idle()); x5
            x4 = (A2: Idle()); x6
            x5 + x6 = x7
            x7 = end
        in [true] x0
    '''

def funny_fine():
    # needs processes C, A
    return ''' G =
        def x0 = C -> A: msg(); x1
            x1 = C -> A: msg(); x2
            x2 = end
        in [true] x0
    '''

def causal_loop_err():
    # the problem here is when we stay in the loop: there is C->A2 (previous iteration) and A1->A2 (next iteration) wich should conflict at A2
    # needs processes C, A1, A2
    return ''' G =
        def x0 = C -> A1: msg(); x1
            x1 + x6 = x2
            x2 = [C_x >= 0] x3 + [C_x <= 0] x7
            x3 = A1 -> A2: msg(); x4
            x4 = (C: idle(), A1: idle(), A2: idle()); x5
            x5 = C -> A2: msg(); x6
            x7 = end
        in [true] x0
    '''

def causal_independent_err():
    # needs processes A, B, C, D
    return ''' G =
        def x0 = A -> B: msg(); x1
            x1 = C -> D: msg(); x2
            x2 = end
        in [true] x0
    '''

def nomraliztion_err():
    # not correct because it mixes internal and external choice when removing ||
    return ''' G =
        def x0 = x1 || x2
            x1 = C -> A1: msg(); x3
            x3 = A1 -> C: msg(); x4
            x2 = C -> A2: msg(); x5
            x5 = A2 -> C: msg(); x6
            x4 || x6 = x7
            x7 = end
        in [true] x0
    '''

def run(ch, components = None, shouldSucceed = True, debug = False):
    try:
        visitor = Projection()
        visitor.execute(ch, components, debug)
        if (components != None):
            chor = visitor.choreography
            vectorize(chor, components)
            processes = components.allProcesses()
            for p in processes:
                visitor.choreography = deepcopy(chor)
                proj = visitor.project(p.name(), p, debug)
                if debug:
                    print("== Projection ==")
                    print(proj)
        passed = True
    except Exception as e:
        passed = False
        if shouldSucceed:
            raise e
    if passed and not shouldSucceed:
        raise Exception("test passed but should have failed")

class ChoreograhyTests(unittest.TestCase):

    def test_fetch(self):
        run(cartAndArmFetch(), cartAndArmWorld())

    def test_handover(self):
        run(armsHandover(), armsHandoverWorld())

    def test_sorting(self):
        run(binSorting(), binSortingWorld())

    def test_ferry(self):
        run(ferry(), ferryWorld(), debug = False)

    def test_ferry1(self):
        run(ferry1(), ferryWorld(), debug = True)

    def test_err1(self):
        run(funny_thread_partition(), shouldSucceed = False)

    def test_ok1(self):
        run(causal_ok())

    def test_err2(self):
        run(causal_err(), shouldSucceed = False)

    def test_err3(self):
        run(causal_independent_err(), shouldSucceed = False)
    
    def test_err4(self):
        run(causal_loop_err(), shouldSucceed = False, debug = False)

    def test_err5(self):
        run(thread_partition_err1(), shouldSucceed = False, debug = False)

    def test_err6(self):
        run(thread_partition_err2(), shouldSucceed = False)

    def test_err7(self):
        run(thread_partition_err3(), shouldSucceed = False)

    def test_funny_causal(self):
        run(funny_fine(), shouldSucceed = True)


if __name__ == '__main__':
    unittest.main()
