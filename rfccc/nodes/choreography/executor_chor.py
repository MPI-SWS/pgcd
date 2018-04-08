import inspect
import threading
import math
import itertools

from parser_chor import *
from ast_chor import *
from ast_proj import *


class ChoreographyExecutor:

    def __init__(self):
        self.parser = ChoreographyParser()
        self.choreography = None
        self.artificial_nodes_counter = -1

    def get_artificial_name(self):
        self.artificial_nodes_counter += 1
        return '__x__' + self.artificial_nodes_counter

    def execute(self, code):
        self.choreography = self.parser.parse(code)
        return self.choreography

    def project(self, proj_name, process):
        projection, state_to_node = CreateProjectionFromChoreography(self.choreography, proj_name, process)
        return self.normalize_projection(projection.start_state[0], state_to_node)

    # for the normalization
    def removeIndirections(self, choreography, state_to_node):
        removedStates = set()
        substitution = {}
        # first scan to figure out what to remove
        for node in state_to_node.values():
            if isinstance(node, Indirection):
                src = node.start_state[0]
                trg = node.end_state[0]
                removedStates.add(src)
                substitution = { k : (trg if v == src else v)  for k, v in substitution }
                substitution[src] = trg
        # now do the remove
        for s in removedStates:
            state_to_node.pop(s)
            choreography.start_state = substitution.get(choreography.start_state, choreography.start_state)
        for node in state_to_node.values:
            node.start_state = [ substitution.get(s, s) for s in node.start_state ]
            node.end_state = [ substitution.get(s, s) for s in node.end_state ]
            if isinstance(node, GuardedChoice):
                for g in node.guarded_states:
                    g.id = substitution.get(g.id, g.id)

    # for the normalization
    def removeForkJoin(self, choreography, state_to_node):
        # if there are nested forks then we should work inside out
        def noForkBeforeJoin(state):
            node = state_to_node[state]
            if isinstance(node, GuardedChoice):
                return all([noForkBeforeJoin(g.id) for g in node.guarded_states])
            elif isinstance(node, Fork):
                return False
            elif isinstance(node, Join):
                return True
            else:
                return all([noForkBeforeJoin(s) for s in node.end_state])
        #find the predecessor of a state
        def findPred(state):
            for node in state_to_node.values():
                if isinstance(node, GuardedChoice):
                    for i, val in enumerate(node.guarded_states):
                        if val.id == state:
                            return node, i
                else:
                    for i, val in enumerate(node.end_state):
                        if val == state:
                            return node, i
        # takes a list of threads and generate the permutation of events
        def mergeThread(pred, pred_index, states):
            #first step categorise the next event
            internal = []
            external = []
            merge = []
            motion = []
            send = []
            receive = []
            join = []
            for s in states:
                node = state_to_node[s]
                if isinstance(node, GuardedChoice):
                    internal.add(s)
                elif isinstance(node, ExternalChoice):
                    external.add(s)
                elif isinstance(node, Motion):
                    motion.add(s)
                elif isinstance(node, SendMessage):
                    send.add(s)
                elif isinstance(node, ReceiveMessage):
                    receive.add(s)
                elif isinstance(node, Merge):
                    merge.add(s)
                elif isinstance(node, Join):
                    join.add(s)
                else:
                    raise Exception("mergeThread: " + str(node))
            #then check for conflicts
            if len(send) > 0 and len(receive) > 0:
                raise Exception("mergeThread, send/receive conflit: " + send + ", " + receive)
            #...
            raise NotImplementedError
        # get the fork we need to remove
        forks = [ state for state, node in state_to_node if isinstance(node, Fork) ]
        # the non-nested ones
        nonNestedFork = [ s for s in forks if noForkBeforeJoin(s) ]
        while len(forks) > 0:
            assert len(nonNestedFork) > 0
            for s in nonNestedFork:
                pres, idx = findPred(s)
                path = mergeThreads(pred, idx, state_to_node[s].end_state)
                raise NotImplementedError("connect back the merged paths")
            # update work list
            forks = [ s for s in forks if not s in nonNestedFork ]
            nonNestedFork = []

    def normalize_projection(self, state, state_to_node, fork_nodes=None):

        node = state_to_node[state]
        if fork_nodes is not None and not (isinstance(node, Fork) or isinstance(node, Join) or isinstance(node, Merge)):
            fork_nodes.append(node)

        if isinstance(node, Indirection):
            state_to_node[node.start_state[0]] = state_to_node[node.end_state[0]]
            self.normalize_projection(node.end_state[0], state_to_node, fork_nodes)
            return

        elif isinstance(node, GuardedChoice):
            for st in node.end_state:
                self.normalize_projection(st, state_to_node, fork_nodes)
            return


        elif isinstance(node, ExternalChoice):
            node_type = state_to_node[node.end_state[0]].tip
            nodes = []
            prev = None
            eq_types = True
            for choice in node.end_state:
                nodes.add(state_to_node[choice])
                # shift delay check, node has to have only one end_state
                if (prev is not None and \
                 not prev.shift_delay_check(state_to_node[choice])) or \
                 len(state_to_node[choice].end_state) != 1:
                    eq_types = False
                    break
                prev = state_to_node[choice]
            # check if merge occured
            if len(nodes) == 1 and nodes[0].tip == Type.merge and len(
                    set(node.start_state) - set(nodes[0].end_state)) == 0:
                state_to_node[state] = Indirection([state], [nodes[0].end_state])
            # if equal types reduce them
            elif eq_types:  # x0 = x1 & x2 | x1 = Y; x3 | x2 = Y; x4  --> example in the paper
                merge_list = [x.end_state[0] for x in nodes]  # x3, x4
                modif = nodes[0]  # x1 = Y; x3
                modif.start_state = node.start_state[0]  # x0 = Y; x3
                modif.end_state = node.end_start[0]  # x0 = Y; x1
                state_to_node[state] = modif  # x0 -> x0 = Y; x1
                # --------------------------------------------------------
                ex_choice = node  # x0 = x1 & x2
                ex_choice.end_state = merge_list  # x0 = x3 & x4
                ex_choice.start_state = modif.end_state  # x1 = x3 & x4
                state_to_node[modif.end_state[0]] = ex_choice  # x1 -> x1 = x3 & x4
            for st in modif.end_state:
                self.normalize_projection(st, state_to_node, fork_nodes)
            return

        elif isinstance(node, Fork):
            branches = []  # array of array of nodes in each branch
            for fork in node.end_state:  # traverse the tree and get all nodes before join
                branch = []
                self.normalize_projection(fork, state_to_node, branch)
                branches.append(branch)

            possible_segments = []  # array of segments of each branch before/with motion primitive
            is_same_node = type(branches[0][0])
            motion = None
            occured_motions = []
            for branch in branches:
                segment = []  # array of nodes occured before motion primitive
                types_before_motion = set()  # check for errors
                for curr_node in branch:
                    if isinstance(curr_node, Motion):
                        occured_motions.append(curr_node)
                        break
                    types_before_motion(type())
                    segment.append(curr_node)
                branch = [x for x in branch if x not in segment and x not in occured_motions]
                if len(segment) > 0:
                    possible_segments.append(segment)

            assert not (types_before_motion.__contains__(ReceiveMessage) and
                        types_before_motion.__contains__(SendMessage)), \
                        "Mixing internal and external choices in " + str(node)
            assert not (types_before_motion.__contains__(GuardedChoice) and
                        types_before_motion.__contains__(ExternalChoice)), \
                        "Mixing internal and external choices in " + str(node)

            guard_args = []
            for perm in itertools.permutations(possible_segments):
                start_state = [node.start_state[0]]
                combination = list(itertools.chain.from_iterable(perm))
                guard_args.append(GuardArg(sympify('True'), self.artificial_nodes_counter())) # TODO DZ: put some predicate instead of true?



        elif isinstance(node, Join):
            return




















