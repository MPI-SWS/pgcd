import unittest
import parser
import spin

def progFetchA():
    return '''
        while( true ) {
        A0: receive(m_Idle) {
                case fold() => {
        A1:         m_Fold;
                    send(C, folded, 0);
                }
                case grab(loc) => {
        A2:         m_Grab(loc);
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
        C0: receive(m_Idle){
            case folded() => skip;
        }
        while (sqrt((C_x - 2)**2 + (C_y - 0)**2) > 0.1) {
        C1:    m_MoveFromTo(Pnt(0,0,0), Pnt(2,0,0));
        }
        send(A, grab, Pnt(2.2,0,0));
        C2: receive(m_Idle){
            case grabbed() => skip;
        }
        send(A, fold, 0);
        C3: receive(m_Idle){
            case folded() => skip;
        }
        while (sqrt((C_x)**2 + (C_y - 0)**2) > 0.1) {
        C4:   m_MoveFromTo(Pnt(2,0,0), Pnt(0,0,0));
        }
        send(A, done, 0);
    '''

def run(progs, shouldSucceed = True, debug = False):
    try:
        prser = parser.Parser()
        parsed = { (name, prser.parse(txt)) for name, txt in progs.items() }
        mc = spin.McMessages(parsed, debug)
        result = mc.check()
        return result == shouldSucceed
    except Exception as e:
        if shouldSucceed:
            raise e

class SpinTests(unittest.TestCase):

    def test_01(self):
        self.assertTrue(run({ "A": progFetchA(), "C": progFetchC() }, debug = True))

if __name__ == '__main__':
    unittest.main()
