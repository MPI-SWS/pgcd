import sys
print(sys.version)

import parser as rp
import choreography.executor_chor as exec


def main():
    algorithm1 = ''' G =
        def x0 = C -> A : action(fold) ; x1
            x1 = (C : idle(), A : fold()) ; x2
            x2 = A -> C : state(folded) ; x3
            x3 + x6 = x4
            x4 = [!inReach(target)]x5 + [inReach(target)]x7
            x5 = ( C : moveToward(target), A : idle()) ; x6
            x7 = C -> A : action(grab) ; x8
            x8 = ( C : idle(), A : grab(target)) ; x9
            x9 = A -> C : state(grabbed) ; x10
            x10 = C -> A : action(fold) ; x11
            x11 = ( C : idle(), A : fold()) ; x12
            x12 = A -> C : state(folded) ; x13
            x13 + x16 = x14
            x14 = [!inReach(target)]x15 + [inReach(target)]x17
            x15 = ( C : moveToward(home), A : idle()) ; x16
            x17 = C -> A : state(done) ; x18
            x18 = end
                in [true]x0
        
    '''
    visitor = exec.ChoreographyExecutor()
    visitor.execute(algorithm1)

main()