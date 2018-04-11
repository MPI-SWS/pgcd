import choreography.executor_chor as exec
import unittest
import parser
from refinement import *
from parser_chor import *
from experiments_setups import *
from test.test import cartAndArmFetch, binSorting, armsHandover, ferry
from vectorize_spec import *
from copy import deepcopy

def prog1C():
    return '''
        send(id_A, msg_rotate, angles);
        receive(m_Idle){
            (msg_moveTo, position, { m_MoveToPosition(position) })
        }
    '''

def prog1A():
    return '''
        receive(m_Idle){
            (msg_rotate, angles, { m_Grab(angles) })
        };
        send(id_C, msg_moveTo, position);
        m_Idle
    '''

def chor1():
    return ''' SendReceive =
        def x0 = C -> A : rotate(abc) ; x1
            x1 = (C : Idle(), A : Grab(abc)) ; x2
            x2 = A -> C : moveTo(pos) ; x3
            x3 = (A : Idle(), C : MoveToPosition(pos)) ; x4
            x4 = end
        in [ True ] x0
    '''

def run(ch, components, progs, shouldSucceed = True, debug = False):
    try:
        visitor = exec.ChoreographyExecutor()
        visitor.execute(ch)
        chor = visitor.choreography
        vectorize(chor, components)
        processes = components.allProcesses()
        for p in processes:
            visitor.choreography = deepcopy(chor)
            proj = visitor.project(p.name(), p, debug)
            prser = parser.Parser()
            prog = prser.parse(progs[p.name()])
            ref = Refinement(prog, proj, debug = True)
            if not ref.check():
                return not shouldSucceed
        return shouldSucceed
    except Exception as e:
        if shouldSucceed:
            raise e

class RefinementTests(unittest.TestCase):
    
    def test_01(self):
        self.assertTrue(run(chor1(), cartAndArmWorld(), { "A": prog1A(), "C": prog1C() }))

if __name__ == '__main__':
    unittest.main()
