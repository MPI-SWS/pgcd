import unittest
from choreography.threads import *
from choreography.parser_chor import ChoreographyParser
import experiments_setups
import xp_fetch_01_test
import xp_sorting_test

class ComputeThreadsTests(unittest.TestCase):

    def test_01(self):
        w = experiments_setups.cartAndArmWorld()
        ch = xp_fetch_01_test.choreo_old()
        c = ChoreographyParser(w).parse(ch, check = False)
        ct = ThreadChecks(c, w)
        node_to_tracker = ct.perform()
        processes_ids = { p.name() for p in w.allProcesses() }
        for (n,t) in node_to_tracker.items():
            self.assertEqual( t.processes, processes_ids)

    def test_02(self):
        w = xp_sorting_test.world()
        ch = xp_sorting_test.choreo()
        c = ChoreographyParser(w).parse(ch, check = False)
        ct = ThreadChecks(c, w)
        state_to_tracker = ct.perform()
        self.assertEqual(state_to_tracker['start'].processes, {"arm", "franka", "carrier", "sensor", "producer"})
        self.assertEqual(state_to_tracker['x5'].processes, {"arm", "franka", "carrier", "sensor"})
        self.assertEqual(state_to_tracker['x10a'].processes, {"producer"})
        self.assertEqual(state_to_tracker['x13d2'].processes, {"arm"})


if __name__ == '__main__':
    unittest.main()
