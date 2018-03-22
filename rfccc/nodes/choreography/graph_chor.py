from ast_chor import *

class ChoreographyCheck:

    def __init__(self, state_to_node, start_state):
        self.start_state = start_state
        self.state_to_node = state_to_node
        self.scope = 0
        self.join_scope = {}
        self.check_graph(start_state, set(), start_state)

    def check_graph(self, state, visited, process):

        node = self.state_to_node[state]
        visited.add(state)

        if isinstance(node, Message):
            self.check_graph(node.end_state[0], visited, process)

        elif isinstance(node, Motion):
            self.check_graph(node.end_state[0], visited, process)

        elif isinstance(node, Guard):
            for s in node.end_state:
                if not visited.__contains__(s):
                    self.check_graph(s, visited, process)
                    break

        elif isinstance(node, Merge):
            self.check_graph(node.end_state[0], visited, process)

        elif isinstance(node, Fork):
            self.scope += 1
            self.join_scope[self.scope] = node.end_state[:]
            for s in node.end_state:
                if not visited.__contains__(s):
                    self.check_graph(s, set(), s)
                    break

        elif isinstance(node, Join):
            if not self.join_scope[self.scope].__contains__(process):
                raise Exception('Cannot execute join whose states are not in the same scope.')
            self.join_scope[self.scope].remove(process)
            if len(self.join_scope[self.scope]) == 0:
                self.check_graph(node.end_state[0], visited, process)

        elif isinstance(node, End):
            print('Test passed... ✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓')



