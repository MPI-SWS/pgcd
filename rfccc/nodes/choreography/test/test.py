import sys

print(sys.version)

import parser as rp
import choreography.executor_chor as exec
import unittest


# The example from the paper
# origin is (0,0), target is (2,0)
def cartAndArmFetch():
    return ''' Fetch =
        def x0 = C -> A : action(fold) ; x1
            x1 = (C : idle(), A : fold()) ; x2
            x2 = A -> C : state(folded) ; x3
            x3 + x6 = x4
            x4 = [sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1] x5 + [sqrt((C_x - 2)**2 + (C_y - 0)**2) <= 0.1] x7
            x5 = (C : moveToward(0,0, 2,0), A : idle()) ; x6
            x7 = C -> A : action(grab) ; x8
            x8 = ( C : idle(), A : grab(target)) ; x9
            x9 = A -> C : state(grabbed) ; x10
            x10 = C -> A : action(fold) ; x11
            x11 = (C : idle(), A : fold()) ; x12
            x12 = A -> C : state(folded) ; x13
            x13 + x16 = x14
            x14 = [sqrt((C_x - 0)**2 + (C_y - 0)**2) > 0.1] x15 + [sqrt((C_x - 0)**2 + (C_y - 0)**2) <= 0.1] x17
            x15 = (C : moveFromTo(2, 0, 0, 0), A : idle()) ; x16
            x17 = C -> A : state(done) ; x18
            x18 = end
        in [true]x0
    '''


# 2 arms next to each other handing an object to each other
# `loc` needs to be within the reach of both arms
# `delta` is a positive offset which is roughly the reach/length of the gripper also is has to be aligned with the position of A and B
def armsHandover():
    return ''' Handover =
        def x0 = A -> B : meetAt(loc); x1
            x1 = (A: moveTo(loc - delta), B: moveTo(loc + delta)); x2
            x2 = A -> B : holding(); x3
            x3 = (A: idle(), B: closeGripper()); x4
            x4 = B -> A : holding(); x5
            x5 = (A: openGripper(), B: idle()); x6
            x6 = A -> B : action(released); x7
            x7 = (A: moveToOrigin(), B: moveToOrigin()); x8
            x8 = end
        in [true] x0
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
# The unlabeled boxes next to the arm is where they get their respective objects from.
# FIXME DZ: this one is syntactically correct but I don't think it fits our sntax restriction unless we have a quite fancy thread correctness check (need to think some more about that)
def binSorting():
    return ''' BinSorting =
        def x0 = (A: idle(), B: idle()); x1         # because we cannot loop to x0 (is that restriction really needed?)
            x1 + x31 + x32 + x33 + x34 + x35 = x2

            x2 = [b0]x3 + [b1]x4 + [b2]x5       # B makes a choice whether is it going to put an object in a bin (b0/1/2 are dummy variables to track the choice back to B)
            x3 = B -> A : useBin(0); x6
            x4 = B -> A : useBin(1); x7
            x5 = B -> A : useBin(2); x8

            x6 = [a0]x10 + [a1]x11 + [a2]x12    # choice at A
            x7 = [a0]x13 + [a1]x14 + [a2]x15    # choice at A
            x8 = [a0]x16 + [a1]x17 + [a2]x18    # choice at A

            x11 + x14 = _x11
            x12 + x15 + x18 = _x12

            x10 = A -> B : done(); x20
            _x11 = A -> B : wait(); x21
            _x12 = A -> B : wait(); x22
            x13 = A -> B : ok(); x23
            x16 = A -> B : ok(); x24
            x17 = A -> B : ok(); x25

            x20 = end
            x21 = (A: putInBin(1), B: idle()); x31
            x22 = (A: putInBin(2), B: idle()); x32
            x23 = (A: idle(), B: putInBin(1)); x33
            x24 = (A: idle(), B: putInBin(2)); x34
            x25 = (A: putInBin(1), B: putInBin(2)); x35

        in [true] x0
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
        def x0 = (A: idle(), B: idle(), C: idle()); ca
            # x1 + cb2a1 = ca
            ca = ca1 || ca3
            ca1 = (B: idle()); ca2
            ca3 = C -> A : rdy(); ca4
            ca4 = (A: putOnCart(), C: idle()); ca5
            ca5 = A -> C : done(); ca6
            ca2 || ca6 = ca2b
            ca2b = (A: idle(), B: idle(), C: moveToB()); cb
            cb = cb1 || cb3
            cb1 = (A: idle()); cb2
            cb3 = C -> B : rdy(); cb4
            cb4 = (A: getFromCart(), C: idle()); cb5
            cb5 = B -> C : done(); cb6
            cb2 || cb6 = cb2a
            cb2a = (A: idle(), B: idle(), C: moveToA()); ca2b1
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

def funny_fine_but_not_causal():
    # DZ: our definition of causality is a bit too strong and we reject example like this which are fine
    # DZ: need to think about something better
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
            x2 = [True] x3 + [True] x7
            x3 = A1 -> A2: msg(); x4
            x4 = (C: idle(), A1: idle(), A2: idle()); x5
            x5 = C -> A2: msg(); x6
            x7 = end
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

def run(ch, shouldSucceed = True):
    try:
        visitor = exec.ChoreographyExecutor()
        visitor.execute(ch)
        passed = True
    except Exception as e:
        passed = False
        if shouldSucceed:
            raise e
    if passed and not shouldSucceed:
        raise Exception("test passed but should have failed")

class ChoreograhyTests(unittest.TestCase):

    if len(exec.Choreography.initialized_components) == 0:
        print('WARNING: no components initialized, this is only for debugging purposes...')

    def test_fetch(self):
        run(cartAndArmFetch())

    def test_handover(self):
        run(armsHandover())

    def test_sorting(self):
        run(binSorting())

    def test_ferry(self):
        run(ferry())

    def test_err1(self):
        run(funny_thread_partition(), False)

    def test_ok1(self):
        run(causal_ok())

    def test_err2(self):
        run(causal_err(), False)

    def test_err2(self):
        run(causal_loop_err(), False)

    def test_funny_causal(self):
        run(funny_fine_but_not_causal(), False)


if __name__ == '__main__':
    unittest.main()
