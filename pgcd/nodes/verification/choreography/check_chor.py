from ast_chor import *
from compatibility import choiceAt
from utils.causality_tracker import *
from threads import *
from spec import DurationSpec

class ChoreographyCheck:

    def __init__(self, chor, world, debug = False):
        self.debug = debug
        self.chor = chor
        self.state_to_node = chor.mk_state_to_node()
        self.scope = 0
        self.process_motions_dictionary = {self.chor.start_state: set()}
        self.join_visited = {}
        self.join_node = {}
        self.join_causalities = {}
        self.motion_check = {}
        self.looped_states = set()
        self.loop_has_motion = False
        self.comps = set()
        self.world = world
        if world != None:
            self.comps = { p.name() for p in world.allProcesses() }
        else:
            self.comps = chor.getProcessNames()
        self.threadTrackers = None
        self.causality = CausalityTracker(self.comps)

    def check_well_formedness(self):
        self.syntacic_checks()
        self.thread_checks()
        if self.debug:
            print("causality, local choice, connectedness, and ... checks")
        self.traverse_graph(self.chor.start_state, set(), self.chor.start_state, self.causality)

    def thread_checks(self):
        if self.debug:
            print("thread checks")
        if self.world != None:
            tc = ThreadChecks(self.chor, self.world.allProcesses())
        else:
            tc = ThreadChecks(self.chor, self.chor.getProcessNames())
        self.threadTrackers = tc.perform(self.debug)
        if self.debug:
            print("thread tracker result:")
        self.chor.state_to_processes = dict()
        for k,v in self.threadTrackers.items():
            self.chor.state_to_processes[k] = v.processes
            if self.debug:
                print("\t" + k + ":\t" + ", ".join(v.processes))

    def syntacic_checks(self):
        if self.debug:
            print("Syntacic checks")
        # at most one end state
        ends = [ s for s in self.chor.statements if s.tip == Type.end ]
        if len(ends) > 1:
            raise Exception("Error: 'end' appeared more than once: " + str(ends))
        # start is defined
        start1 = [ s for s in self.chor.statements if self.chor.start_state in s.start_state ]
        if len(start1) != 1:
            raise Exception("Error: start state not exactly once on the LHS: " + str(start1))
        start2 = [ s for s in self.chor.statements if self.chor.start_state in s.end_state ]
        if len(start2) > 0:
            raise Exception("Error: start state on the RHS: " + str(start2))
        # each non-start state at exactly once on the RHS and LHS
        left_states = set()
        right_states = set()
        for s in self.chor.statements:
            for start in s.start_state:
                if start in left_states:
                    raise Exception("Error: " + str(start) + " appears more than once on the LHS")
                else:
                    left_states |= {start}
            for end in s.end_state:
                if end in right_states:
                    raise Exception("Error: " + str(end) + " appears more than once on the RHS")
                else:
                    right_states |= {end}
        assert not len(left_states ^ right_states) != 1, 'States ' + str((left_states ^ right_states) - {self.chor.start_state}) + ' are not on LHS or RHS!'

    def choice_at(self, node):
        if self.world != None:
            # the proper way
            p = choiceAt(node, self.world.allProcesses())
            return p.name()
        else:
            # poor man's version
            candidates = { str(s).split('_')[0] for gs in node.guarded_states for s in gs.expression.free_symbols }
            ps = candidates & self.chor.getProcessNames()
            assert len(ps) == 1, "choice not local: " + str(node)
            return ps.pop()

    def traverse_graph(self, state, visited, process, causality):

        node = self.state_to_node[state]
        visited.add(state)

        if self.debug:
            print("visiting: ", causality.time, node)

        if isinstance(node, Message):
            #TODO the message sequence must touch all the process in the thread
            causality.p_message_q(node.sender, node.receiver, node.start_state)
            self.traverse_graph(node.end_state[0], visited, process, causality)
            return

        elif isinstance(node, Motion):
            #TODO do the intersection of the durations and check one non-interruptible if next op is send (if join then check accross branches), the non-interruptible is the sender
            assert len(node.motions) > 0
            d = DurationSpec()
            for comp_mot in node.motions:
                name = comp_mot.id
                self.process_motions_dictionary[process].add(name)
                self.comps -= {name}
                p = self.chor.getProcess(name)
                if p != None:
                    motion = p.motionPrimitive(comp_mot.mp_name, *(comp_mot.mp_args))
                    d2 = motion.duration()
                    d = d.intersect(d2)
                else:
                    print("could not resolve", comp_mot)
            self.loop_has_motion = True
            causality.motion(d)
            self.traverse_graph(node.end_state[0], visited, process, causality)
            return

        elif isinstance(node, GuardedChoice):
            #TODO send/receive same for all after that
            p = self.choice_at(node)
            causality.choice_at_p(p)
            self.check_same_path_twice(node, visited, process, causality)
            return

        elif isinstance(node, Merge):
            self.check_loop_had_motion(visited, node)
            self.traverse_graph(node.end_state[0], visited, process, causality)
            return

        elif isinstance(node, Fork):
            self.check_each_process_and_set_check_vars(node, visited, causality)
            return

        elif isinstance(node, Join):
            #TODO send right after join and if motions before the join, one is non-interruptible
            if node not in self.join_visited:
                self.join_visited[node] = len(node.start_state) -1
                self.join_causalities[node] = causality
            else:
                self.join_visited[node] -= 1
                self.join_causalities[node] = self.join_causalities[node].join(causality)
            if self.join_visited[node] == 0:
                self.traverse_graph(node.end_state[0], visited, process, self.join_causalities[node])
            return

        elif isinstance(node, End): #TODO no supported for the moment
            return
        
        self.check_no_disconnected_parts(visited)
        self.check_every_process_has_motion_in_one_thread()
        if self.debug:
            print('---> Test passed ✓✓✓')

    def check_same_path_twice(self, node, visited, process, causality):
        for s in node.end_state:
            # TODO self.causality.GUARD?
            if not visited.__contains__(s):
                self.traverse_graph(s, visited, process, causality)
                break
            # If state occurred again see if loop has necessary things
            elif not self.looped_states.__contains__(s):
                self.looped_states.add(s)
                self.traverse_graph(s, visited, process, causality)
                break
        return

    def check_no_disconnected_parts(self, visited):
        assert not len(self.state_to_node) != len(visited), 'There are some disconnected graph parts!'

    def check_component_motion_is_in_only_one_forked_thread(self):
        for i in range(0, len(self.motion_check[self.scope]) - 1):
            for j in range(i, len(self.motion_check[self.scope]) - 1):
                A = self.process_motions_dictionary[self.motion_check[self.scope][i]]
                B = self.process_motions_dictionary[self.motion_check[self.scope][j]]
                self.comps -= (A | B)
                assert not len(A & B) > 0, 'Motion primitive used in parallel processes: "' + \
                                           self.motion_check[self.scope][
                                               i] + '" and "' + self.motion_check[self.scope][j] + '".'

    def check_loop_had_motion(self, visited, node):
        assert not((not self.loop_has_motion) and visited.__contains__(node.end_state[0])), 'Loop "' + ','.join(
            node.start_state) + '" has not any motion!'
        self.loop_has_motion = False

    def check_each_process_and_set_check_vars(self, node, visited, causality):
        self.scope += 1
        self.motion_check[self.scope] = node.end_state[:]
        for s in node.end_state:
            var = causality.fork_new_thread()
            if not visited.__contains__(s):
                if not self.process_motions_dictionary.__contains__(s):
                    self.process_motions_dictionary[s] = set()
                self.traverse_graph(s, visited, s, var)

    def check_every_process_has_motion_in_one_thread(self):
        assert not len(self.comps) > 0, 'No motions found for: ' + ','.join(self.comps) + '.'


