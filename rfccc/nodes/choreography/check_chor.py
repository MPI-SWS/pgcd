from ast_chor import *


class ChoreographyCheck:

    def __init__(self, state_to_node, start_state):
        self.start_state = start_state
        self.state_to_node = state_to_node
        self.scope = 0
        self.forks = 0
        self.thread_motions_dictionary = {start_state: set()}
        self.join_scope = {}
        self.join = None
        self.motion_check = {}
        self.loop_has_motion = True

    def check_well_formedness(self):
        try:
            self.traverse_graph(self.start_state, set(), self.start_state)
            return True
        except Exception as e:
            print(str(e))
            return False

    def traverse_graph(self, state, visited, process):

        node = self.state_to_node[state]
        visited.add(state)

        if isinstance(node, Message):
            self.traverse_graph(node.end_state[0], visited, process)
            return

        elif isinstance(node, Motion):
            for comp_mot in node.motions:
                self.thread_motions_dictionary[process].add(comp_mot.id)
            self.loop_has_motion = True
            self.traverse_graph(node.end_state[0], visited, process)
            return

        elif isinstance(node, GuardedChoice):
            for s in node.end_state:
                if not visited.__contains__(s):
                    self.traverse_graph(s, visited, process)
                    break
            return


        elif isinstance(node, Merge):
            self.check_loop_had_motion(visited, node)
            self.traverse_graph(node.end_state[0], visited, process)
            return

        elif isinstance(node, Fork):
            self.check_each_thread_separately_for_correct_join_and_separate_motions(node, visited)
            return

        elif isinstance(node, Join):
            self.check_joining_same_scope_and_threads(process, node, visited)
            return

        elif isinstance(node, End):
            return

        if self.check_all_threads_joined() and self.check_no_disconnected_parts(visited):
            print(' ---> Test passed ✓✓✓')

    def check_no_disconnected_parts(self, visited):
        if len(self.state_to_node) != len(visited):
            raise Exception('There are some disconnected graph parts!')

    def check_joining_same_scope_and_threads(self, process, node, visited):
        if not self.join_scope[self.scope].__contains__(process):
            raise Exception('Cannot execute join whose states are not in the same scope.')
        if self.join is None:
            self.join = node
        elif self.join != node:
            raise Exception('Cannot join these processes: "' + ','.join(node.start_state) + '".')

        self.join_scope[self.scope].remove(process)
        if len(self.join_scope[self.scope]) == 0:
            del self.join_scope[self.scope]
            del self.motion_check[self.scope]
            self.scope -= 1
            self.join = None
            self.traverse_graph(node.end_state[0], visited, process)

    def check_component_move_in_only_one_forked_thread(self):
        for i in range(0, len(self.motion_check[self.scope]) - 1):
            for j in range(i, len(self.motion_check[self.scope]) - 1):
                A = self.thread_motions_dictionary[self.motion_check[self.scope][i]]
                B = self.thread_motions_dictionary[self.motion_check[self.scope][j]]
                if len(A & B) > 0:
                    raise Exception('Motion primitive used in parallel threads: "'
                                    + self.motion_check[self.scope][i] + '" and "'
                                    + self.motion_check[self.scope][j] + '".')

    def check_all_threads_joined(self):
        if len(self.join_scope) != 0:
            raise Exception('Failed to join all threads!')

    def check_loop_had_motion(self, visited, node):
        if not self.loop_has_motion and visited.__contains__(node.end_state[0]):
            raise Exception('Loop has not any motion!')
        self.loop_has_motion = False

    def check_each_thread_separately_for_correct_join_and_separate_motions(self, node, visited):
        self.scope += 1
        self.forks += 1
        self.join_scope[self.scope] = node.end_state[:]
        self.motion_check[self.scope] = node.end_state[:]
        for s in node.end_state:
            if not visited.__contains__(s):
                if not self.thread_motions_dictionary.__contains__(s):
                    self.thread_motions_dictionary[s] = set()
                self.traverse_graph(s, visited, s)
