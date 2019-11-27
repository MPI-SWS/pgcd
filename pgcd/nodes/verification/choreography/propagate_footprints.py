from ast_chor import *
from sympy import *

# assume thread check passed
class PropagateFootprint():

    def __init__(self, chor, debug = False):
        self.done = False
        self.visited = set()
        self.chor = chor
        self.state_to_node = chor.mk_state_to_node()
        self.state_to_footprints = dict()
        self.debug = debug

    def perform(self):
        if not self.done:
            self._do(self.chor.start_state, [Footprint(Symbol('fpx'), Symbol('fpy'), Symbol('fpz'), sympify(True))])
            self.chor.state_to_footprints = self.state_to_footprints
            if self.debug:
                for k, v in self.state_to_footprints.items():
                    print(k, v)

    def _do(self, state, stack):
        if state in self.visited:
            assert self.state_to_footprints[state] == stack[0]
        else:
            self.visited.add(state)
            self.state_to_footprints[state] = stack[0]
            node = self.state_to_node[state]
            if isinstance(node, Fork):
                for e, fp in zip(node.end_state, node.footprints):
                    self._do(e, [fp] + stack)
            elif isinstance(node, Join):
                new_stack = stack[1:]
                self._do(node.end_state[0], new_stack)
            else:
                for e in node.end_state:
                    self._do(e, stack)
