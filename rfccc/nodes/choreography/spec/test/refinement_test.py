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

def progFetchA():
    return '''
        while( true ) {
            receive(m_Idle) {
                (msg_fold, dummy, {
                    m_Fold;
                    send(id_C, msg_folded, 0)
                }),
                (msg_grab, loc, {
                    m_Grab(loc);
                    send(id_C, msg_grabbed, 0)
                }),
                (msg_done, dummy, {
                    exit(0) 
                })
            }
        }
    '''

def progFetchC():
    return '''
        send(id_A, msg_fold, 0);
        receive(m_Idle){
            (msg_folded, dummy, { skip })
        };
        while (sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1) {
            m_MoveFromTo(Pnt(0,0,0), Pnt(2,0,0))
        };
        send(id_A, msg_grab, Pnt(2.2,0,0));
        receive(m_Idle){
            (msg_grabbed, dummy, { skip })
        };
        send(id_A, msg_fold, 0);
        receive(m_Idle){
            (msg_folded, dummy, { skip })
        };
        while (sqrt((C_x)**2 + (C_y - 0)**2) > 0.1) {
            m_MoveFromTo(Pnt(2,0,0), Pnt(0,0,0))
        };
        send(id_A, msg_done, 0)
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

    def test_02(self):
        self.assertTrue(run(cartAndArmFetch(), cartAndArmWorld(), { "A": progFetchA(), "C": progFetchC() }))

if __name__ == '__main__':
    unittest.main()
