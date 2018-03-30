from ast_chor import *
import numpy as np


class ChoreographyCheck:

    def __init__(self, state_to_node, start_state):
        self.start_state = start_state
        self.state_to_node = state_to_node
        self.scope = 0
        self.forks = 0
        self.process_motions_dictionary = {start_state: set()}
        self.join_scope = {}
        self.join_node = {}
        self.join_causalities = {}
        self.motion_check = {}
        self.loop_has_motion = True
        self.comps = set(Choreography.initialized_components)
        self.causality = CausalityTracker(Choreography.initialized_components)

    def check_well_formedness(self):
        try:
            self.traverse_graph(self.start_state, set(), self.start_state, self.causality)
            return True
        except Exception as e:
            print(str(e))
            return False

    def traverse_graph(self, state, visited, process, causality):

        node = self.state_to_node[state]
        visited.add(state)

        if isinstance(node, Message):
            causality.p_message_q(node.comp1, node.comp2)
            self.traverse_graph(node.end_state[0], visited, process, causality)
            return

        elif isinstance(node, Motion):
            for comp_mot in node.motions:
                self.process_motions_dictionary[process].add(comp_mot.id)
            self.loop_has_motion = True
            # TODO self.causality.motion( DURATION? )
            self.traverse_graph(node.end_state[0], visited, process, causality)
            return

        elif isinstance(node, GuardedChoice):
            for s in node.end_state:
                # TODO self.causality.GUARD?
                if not visited.__contains__(s):
                    self.traverse_graph(s, visited, process, causality)
                    break
            return

        elif isinstance(node, Merge):
            self.check_loop_had_motion(visited, node)
            self.traverse_graph(node.end_state[0], visited, process, causality)
            return

        elif isinstance(node, Fork):
            self.check_each_process_and_set_check_vars(node, visited, causality)
            return

        elif isinstance(node, Join):
            self.check_joining_same_scope_and_threads(process, node, visited, causality)
            return

        elif isinstance(node, End):
            return

        if self.check_all_threads_joined() and self.check_no_disconnected_parts(
                visited) and self.check_every_process_has_motion_in_one_thread():
            print(' ---> Test passed ✓✓✓')

    def check_no_disconnected_parts(self, visited):
        if len(self.state_to_node) != len(visited):
            raise Exception('There are some disconnected graph parts!')

    def check_joining_same_scope_and_threads(self, process, node, visited, causality):
        if not self.join_scope[self.scope].__contains__(process):
            raise Exception('Cannot execute join whose states are not in the same scope.')
        if self.join_node[self.scope] is None:
            self.join_node[self.scope] = node
        elif self.join_node[self.scope] != node:
            raise Exception('Cannot join these threads: "' + ','.join(node.start_state) + '".')

        self.join_causalities[self.scope].append(causality)
        self.join_scope[self.scope].remove(process)
        if len(self.join_scope[self.scope]) == 0:
            del self.join_scope[self.scope]
            del self.motion_check[self.scope]
            self.scope -= 1
            self.join_node[self.scope] = None
            causality.join_with_causalities(self.join_causalities[self.scope])
            del self.join_causalities[self.scope]
            self.traverse_graph(node.end_state[0], visited, process, causality)

    def check_component_motion_is_in_only_one_forked_thread(self):
        for i in range(0, len(self.motion_check[self.scope]) - 1):
            for j in range(i, len(self.motion_check[self.scope]) - 1):
                A = self.process_motions_dictionary[self.motion_check[self.scope][i]]
                B = self.process_motions_dictionary[self.motion_check[self.scope][j]]
                self.comps -= (A | B)
                if len(A & B) > 0:
                    raise Exception('Motion primitive used in parallel processes: "'
                                    + self.motion_check[self.scope][i] + '" and "'
                                    + self.motion_check[self.scope][j] + '".')

    def check_all_threads_joined(self):
        if len(self.join_scope) != 0:
            raise Exception('Failed to join all processes!')

    def check_loop_had_motion(self, visited, node):
        if not self.loop_has_motion and visited.__contains__(node.end_state[0]):
            raise Exception('Loop has not any motion!')
        self.loop_has_motion = False

    def check_each_process_and_set_check_vars(self, node, visited, causality):
        self.scope += 1
        self.forks += 1
        self.join_scope[self.scope] = node.end_state[:]
        self.join_causalities[self.scope] = []
        self.motion_check[self.scope] = node.end_state[:]
        for s in node.end_state:
            if not visited.__contains__(s):
                if not self.process_motions_dictionary.__contains__(s):
                    self.process_motions_dictionary[s] = set()
                self.traverse_graph(s, visited, s, causality.fork_new_thread())

    def check_every_process_has_motion_in_one_thread(self):
        if len(self.comps) > 0:
            raise Exception('No motions found for: ' + ','.join(self.comps) + '".')


class CausalityTracker:

    def __init__(self, process_set):
        self.time = 0
        self.process_to_vclock = {}
        self.process_set = process_set
        self.init_vclocks()

    def init_vclocks(self):
        idx = 0
        for proc in self.process_set:
            self.process_to_vclock[proc] = (np.zeros((len(self.process_set),), dtype=int), idx)
            idx += 1

    def inc_thread_vclock(self, p):
        self.process_to_vclock[p][0][self.process_to_vclock[p][1]] += 1

    def p_concurrent_q(self, p, q):
        return not (self.p_after_q(p, q) or self.p_after_q(q, p))

    def p_before_q(self, p, q):
        return self.p_after_q(q, p)

    def p_after_q(self, p, q):
        v1 = self.process_to_vclock[p][0]
        v2 = self.process_to_vclock[q][0]
        if len(v1) > len(v2):
            return False
        if v1 == v2:
            return False
        for first, second in zip(v1, v2):
            if first > second:
                return False

        return True

    def p_message_q(self, p, q):
        assert self.p_after_q(p, q)
        self.inc_thread_vclock(p)
        self.process_to_vclock[q][0] = np.copy(self.process_to_vclock[p][0])
        self.inc_thread_vclock(q)

    def motion(self, duration):
        self.time += duration
        self.init_vclocks()

    def join_with_causalities(self, causalities):
        for i in range(len(causalities)):
            assert self.time == causalities[i].time
            for proc in self.process_set:
                self.process_to_vclock[proc][0] = np.maximum(self.process_to_vclock[proc][0],
                                                             causalities[i].thread_to_vclock[proc][0])

    def fork_new_thread(self):
        cl = CausalityTracker(self.process_set)
        cl.time = self.time
        idx = 0
        for proc in self.process_set:
            cl.process_to_vclock[proc] = (np.copy(self.process_to_vclock[proc][0]), idx)
            idx += 1
        return cl
