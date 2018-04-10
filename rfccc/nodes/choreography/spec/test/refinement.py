import parser as rp
import choreography.executor_chor as exec
import unittest
from experiments_setups import *
from vectorize_spec import *
from copy import deepcopy

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
            ref = Refinement(progs[p], proj)
            return shouldSucceed == ref.check()
    except Exception as e:
        if shouldSucceed:
            raise e

class RefinementTests(unittest.TestCase):
    
    def test_01(self):
        assert False
