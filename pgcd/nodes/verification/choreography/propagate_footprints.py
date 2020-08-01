from ast_chor import *
from sympy import *
from spec.contract import *
import logging

log = logging.getLogger("PropagateFootprint")

# assume thread check passed
class PropagateFootprint():

    def __init__(self, chor):
        self.done = False
        self.visited = set()
        self.chor = chor
        self.state_to_node = chor.mk_state_to_node()
        self.state_to_contracts = dict()

    def perform(self):
        if not self.done:
            world = self.chor.world
            assert(world != None)
            defaultContract = FpContract("default contract", world.allProcesses(), world.frame(), 
                                         Symbol('fpx'), Symbol('fpy'), Symbol('fpz'), S.true)
            self._do(self.chor.start_state, [defaultContract])
            self.chor.state_to_contracts = self.state_to_contracts
            if log.isEnabledFor(logging.DEBUG):
                for k, v in self.state_to_contracts.items():
                    log.debug("%s %s", k, v)

    def _do(self, state, stack):
        if state in self.visited:
            assert self.state_to_contracts[state] == stack[0]
        else:
            self.visited.add(state)
            self.state_to_contracts[state] = stack[0]
            node = self.state_to_node[state]
            if isinstance(node, Fork):
                for e, fp in zip(node.end_state, node.footprints):
                    frame = self.chor.world.frame()
                    processes = self.chor.getProcessesAt(e)
                    if isinstance(fp, Footprint):
                        contract = fp.toFpContract(e, processes, frame)
                    else: #already a contract
                        assert(fp.components() == processes)
                        contract = fp
                    self._do(e, [contract] + stack)
            elif isinstance(node, Join):
                new_stack = stack[1:]
                self._do(node.end_state[0], new_stack)
            else:
                for e in node.end_state:
                    self._do(e, stack)
