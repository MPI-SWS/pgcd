import unittest
import parser
import spin
from refinement_test import progFetchA, progFetchC

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
