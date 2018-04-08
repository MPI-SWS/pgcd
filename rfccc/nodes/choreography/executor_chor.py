import inspect
import threading
import math
import itertools
from copy import copy
from sympy import S

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

    def removeUnreachable(self, choreography, state_to_node):
        visited = set()
        frontier = { choreography.start_state }
        while len(frontier) > 0:
            state = frontier.pop()
            if not state in visited:
                visited.add(state)
                node = state_to_node[state]
                for e in node.end_state:
                    frontier.add(e)
        state_to_node = { (k,v) for k,v in state_to_node if k in visited }
        vals = set(state_to_node.values())
        choreography.statements = [ s for s in choreography.statements if s in vals ]

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

    def flattenInternalChoice(self, choreography, state_to_node):
        # assumes no loop without any event
        for n in state_to_node.values():
            if isinstance(n, GuardedChoice):
                n.guarded_states = n.get_successors()
                n.end_state = [ g.id for g in n.guarded_states ]
        self.removeIndirections(choreography, state_to_node)

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
        # find the predecessor of a state
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
        def findMp(state):
            node = state_to_node[state]
            if isinstance(node, Motion):
                return [state]
            else:
                return [ s for succ in node.end_state for s in findMp(succ) ]
        # pull together the motion primitives
        # takes a list of threads and generate the permutation of events
        # TODO diverge in case of loop within fork-join
        def mergeThreads(pred, pred_index, pendingMotion, states):
            #first step categorise the next event
            internal = []
            external = []
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
                    raise Exception("mergeThreads: " + str(node))
            if len(join) > 1:
                raise Exception("mergeThreads ambiguous join " + str(join))
            # skip the merge
            if len(merge) > 0:
                merged = list({ m.end_state[0] for m in merge })
                states = internal + external + motion + send + receive + merged + join
                return mergeThreads(pred, pred_index, pendingMotion, states)
            # merge the internal
            if len(internal) > 0:
                newIntId = self.get_artificial_name()
                guards = [ g for i in internal for g in i.get_successors(state_to_node) ]
                newInt = GuardedChoice([newExtId()], guards)
                internal = [newInt]
                state_to_node[newIntId] = newInt
            # merge the external choices (temporary allow multiple motions in parallel)
            if len(external) + len(receive) + min(1, len(motion)) > 1: #all the motion are 1 choice (TO)
                newExtId = self.get_artificial_name()
                nodes = [ e for n in external for e in n.end_state ] + [ r.start_state[0] for r in receive ] + [ m.start_state[0] for m in motion ]
                newExt = ExternalChoice([newExtId()], nodes)
                external = [newExt]
                receive = []
                motion = []
                state_to_node[newExtId] = newExt
            # check for conflicts
            if len(internal) + len(send) > 0 and len(external) + len(receive) > 0:
                raise Exception("mergeThreads, internal/external choice conflit: " + (internal+send) + ", " + (external+receive))
            # at that point there is not much choice left, we should be either internal, send, external, or motion
            msgAlternatives = internal + send + external + receive
            assert len(msgAlternatives) <= 2
            if len(msgAlternatives) == 0:
                # not 0-time event, let's move or we are done
                if len(motion) > 0:
                    assert len(join) == 0
                    motion.extends(pendingMotion)
                    #TODO a bit too specialized
                    goodOnes = [ m for m in motions if state_to_node[m].motions[0].mp_name != "wait" ]
                    assert(len([goodOnes]) == 1)
                    mp = copy(state_to_node[goodOnes.pop()])
                    newId = self.get_artificial_name()
                    mp.start_state[0] = newId
                    pred.end_state[pred_index] = newId
                    state_to_node[newId] = mp
                    successors = [ end for m in motions for end in state_to_node[m].end_state ]
                    return mergeThreads(mp, 0, [], successors)
                else:
                    assert len(pendingMotion) == 0
                    assert len(join) == 1
                    return [(pred, pred_index)], join.pop()
            elif len(msgAlternatives) == 1:
                a = state_to_node[msgAlternatives.pop()]
                a2 = copy(a)
                newId = self.get_artificial_name()
                a2.start_state[0] = newId
                state_to_node[newId] = a2
                #update the predecessor
                pred.end_state[pred_index] = newId
                if isinstance(pred, GuardedChoice):
                    pred.guarded_states[pred_index].id = newId
                # continue with the rest ...
                acc = []
                join = None
                pendingMP = pendingMotion
                if isinstance(a2, ExternalChoice):
                    mpId = [ s for s in a2.end_state if isinstance(state_to_node[s], Motion) ]
                    if len(mpId) > 1:
                        reprMP = mpId.pop()
                        a2.end_state = [ s for s in a2.end_state if not isinstance(state_to_node[s], Motion) ].append(mpId)
                        pendingMP.extends(mpId)
                for i, e in enumerate(a2.end_state):
                    lst, j = mergeThreads(a2, i, pendingMP, motion + [e] + join)
                    acc.extends(lst)
                    if join is None:
                        join = j
                    else:
                        assert(join == j)
                return acc, join
            else:
                # new choice
                newId = self.get_artificial_name()
                choice = GuardedChoice([newId], [GuardArg(S.true(), newId) for x in msgAlternatives])
                state_to_node[newId] = choice
                #update the predecessor
                pred.end_state[pred_index] = newId
                if isinstance(pred, GuardedChoice):
                    pred.guarded_states[pred_index].id = newId
                # one action
                acc = []
                join = None
                for i, a in msgAlternatives:
                    remaining = copy(msgAlternatives).pop(i)
                    a2 = copy(state_to_node[a])
                    newId2 = self.get_artificial_name()
                    a2.start_state[0] = newId2
                    state_to_node[newId2] = a2
                    choice.end_state[i] = newId2
                    choice.guarded_states[i].id = newId2
                    pendingMP = pendingMotion
                    # if next is motion (receive) only one time thing
                    if isinstance(a2, ExternalChoice):
                        mpId = [ s for s in a2.end_state if isinstance(state_to_node[s], Motion) ]
                        if len(mpId) > 1:
                            reprMP = mpId.pop()
                            a2.end_state = [ s for s in a2.end_state if not isinstance(state_to_node[s], Motion) ].append(mpId)
                            pendingMP.extends(mpId)
                    # continue with the rest ...
                    for i, e in enumerate(a2.end_state):
                        # in case of timee, ignore the non-time events
                        if isinstance(a2, ExternalChoice) and isinstance(state_to_node[e], Motion):
                            remaining = [ mp for r in remaining for mp in findMp(r) ]
                        lst, j = mergeThreads(a2, i, pendingMP, remaining + motion + [e] + join)
                        acc.extends(lst)
                        if join is None:
                            join = j
                        else:
                            assert(join == j)
                return acc, join
        # get the fork we need to remove
        forks = [ state for state, node in state_to_node if isinstance(node, Fork) ]
        # the non-nested ones
        nonNestedFork = [ s for s in forks if noForkBeforeJoin(s) ]
        while len(forks) > 0:
            assert len(nonNestedFork) > 0
            for s in nonNestedFork:
                pred, idx = findPred(s)
                lastIds, join = mergeThreads(pred, idx, [], state_to_node[s].end_state)
                # replace the last join by a merge is needed
                if len(lastIds) == 1:
                    last, idx = lastIds.pop()
                    last.end_state[idx] = state_to_node[join].end_state[0]
                else:
                    merge = Merge([], join.end_state)
                    for pred, index in lastIds:
                        newId = sefl.get_artificial_name()
                        pred.end_state[index] = newId
                        merge.start_state.append(newId)
            # update work list
            forks = [ s for s in forks if not s in nonNestedFork ]
            nonNestedFork = []
            # remove unreachable node
            self.removeUnreachable(choreography, state_to_node)

    def normalize_projection(self, choreography, state_to_node):
        self.removeIndirections(choreography, state_to_node)
        self.removeForkJoin(choreography, state_to_node)
        raise NotImplementedError("coherence check and optionally the delaying of external choice")

