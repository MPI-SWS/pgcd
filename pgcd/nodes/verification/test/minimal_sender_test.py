import unittest
from verification.choreography.threads import ThreadChecks
from verification.choreography.parser_chor import ChoreographyParser
from verification.choreography.minimal_sender import *
from verification.spec.env import Env
import experiments_setups
import xp_fetch_02_test
import xp_sorting_test


class MinimalSenderTests(unittest.TestCase):

    def test_01(self):
        env = Env(experiments_setups.cartAndArmWorld(), xp_fetch_02_test.contracts())
        ch = xp_fetch_02_test.choreo_new()
        c = ChoreographyParser(env).parse(ch, check = False)
        ct = ThreadChecks(c, env)
        node_to_tracker = ct.perform()
        threads = { n:t.processes for (n,t) in node_to_tracker.items() }
        cm = UniqueMinimalSender(c, env, threads)
        state_to_tracker = cm.check()
        for (n,t) in state_to_tracker.items():
            print(n, t)
        self.assertEqual(state_to_tracker['prepare0'].minimalSender, set())
        self.assertEqual(state_to_tracker['prepare0'].receivers, set())
        self.assertEqual(state_to_tracker['split1'].minimalSender, {'C'})
        self.assertEqual(state_to_tracker['split1'].receivers, {'A'})
        self.assertEqual(state_to_tracker['prepare2'].minimalSender, set())
        self.assertEqual(state_to_tracker['prepare2'].receivers, set())
        self.assertEqual(state_to_tracker['split4'].minimalSender, {'A'})
        self.assertEqual(state_to_tracker['split4'].receivers, {'A','C'})

if __name__ == '__main__':
    unittest.main()
