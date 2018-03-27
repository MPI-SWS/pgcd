from ast_chor import *


class ChoreographyCheck:

    def __init__(self, state_to_node, start_state):
        self.start_state = start_state
        self.state_to_node = state_to_node
        self.scope = 0
        self.forks = 0
        self.motion_thread = {start_state: set()}
        self.join_scope = {}
        self.join = None
        self.motion_check = {}
        self.loop_has_motion = True
        self.check_graph(start_state, set(), start_state)

    def check_graph(self, state, visited, process):

        node = self.state_to_node[state]
        visited.add(state)

        if isinstance(node, Message):
            self.check_graph(node.end_state[0], visited, process)

        elif isinstance(node, Motion):
            for comp_mot in node.motions:
                self.motion_thread[process].add(comp_mot.id)

            self.loop_has_motion = True
            self.check_graph(node.end_state[0], visited, process)

        elif isinstance(node, GuardedChoice):
            for s in node.end_state:
                if not visited.__contains__(s):
                    self.check_graph(s, visited, process)
                    break


        elif isinstance(node, Merge):
            if not self.loop_has_motion and visited.__contains__(node.end_state[0]):
                raise Exception('Loop has not any motion!.')
            self.loop_has_motion = False
            self.check_graph(node.end_state[0], visited, process)


        elif isinstance(node, Fork):
            self.scope += 1
            self.forks += 1
            self.join_scope[self.scope] = node.end_state[:]
            self.motion_check[self.scope] = node.end_state[:]
            for s in node.end_state:
                if not visited.__contains__(s):
                    if not self.motion_thread.__contains__(s):
                        self.motion_thread[s] = set()
                    self.check_graph(s, set(), s)

        elif isinstance(node, Join):
            if not self.join_scope[self.scope].__contains__(process):
                raise Exception('Cannot execute join whose states are not in the same scope.')
            if self.join is None:
                self.join = node
            elif self.join != node:
                raise Exception('Cannot join these processes: "' + ','.join(node.start_state) + '".')

            self.join_scope[self.scope].remove(process)
            if len(self.join_scope[self.scope]) == 0:
                for i in range(0, len(self.motion_check[self.scope]) - 1):
                    A = self.motion_thread[self.motion_check[self.scope][i]]
                    B = self.motion_thread[self.motion_check[self.scope][i + 1]]
                    if len(A & B) > 0:
                        raise Exception('Motion primitive used in parallel threads: "'
                                        + self.motion_check[self.scope][i] + '" and "'
                                        + self.motion_check[self.scope][i + 1] + '".')
                del self.join_scope[self.scope]
                del self.motion_check[self.scope]
                self.scope -= 1
                self.check_graph(node.end_state[0], visited, process)
                self.join = None

        elif isinstance(node, End):
            if len(self.join_scope) != 0:
                raise Exception('Failed to join all threads!')
            print('Test passed... ✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓')
