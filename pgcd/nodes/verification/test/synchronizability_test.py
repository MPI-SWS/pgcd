import unittest
from choreography.threads import ThreadChecks
from choreography.parser_chor import ChoreographyParser
from choreography.minimal_sender import UniqueMinimalSender
from choreography.synchronizability import *
from spec.env import Env
import experiments_setups
import xp_fetch_02_test
import xp_sorting_test


class SynchronizabilityTests(unittest.TestCase):

    def test_01(self):
        env = Env(experiments_setups.cartAndArmWorld(), xp_fetch_02_test.contracts())
        ch = xp_fetch_02_test.choreo_new()
        c = ChoreographyParser(env).parse(ch, check = False)
        ct = ThreadChecks(c, env)
        node_to_tracker = ct.perform()
        threads = { n:t.processes for (n,t) in node_to_tracker.items() }
        cm = UniqueMinimalSender(c, env, threads)
        node_to_tracker = cm.check()
        minSender = dict()
        for (n,t) in node_to_tracker.items():
            if len(t.minimalSender) == 1:
                minSender[n] = t.minimalSender.pop()
            else:
                minSender[n] = None
        cs = Synchronizability(c, env, threads, minSender)
        state_to_tracker = cs.check()
        for (n,t) in state_to_tracker.items():
            print(n, t)
        pass

if __name__ == '__main__':
    unittest.main()
