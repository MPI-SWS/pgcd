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
        self.depth = 0

    def get_artificial_name(self):
        self.artificial_nodes_counter += 1
        return '__x__' + str(self.artificial_nodes_counter)

    def execute(self, code):
        self.choreography = self.parser.parse(code)
        return self.choreography

    def project(self, proj_name, process, debug = False):
        if debug:
            print("== Global ==")
            print(self.choreography)
        projection, state_to_node = CreateProjectionFromChoreography(self.choreography, proj_name, process)
        if debug:
            print("-- Raw projection (before normalization) --")
            print(projection)
        try:
            self.normalize_projection(projection, state_to_node, debug)
        except Exception as ex:
            print("ERROR during projection, nodes are")
            for v in state_to_node.values():
                print(v)
            raise
        return projection, state_to_node

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
        toRemove = {k for k in state_to_node if not k in visited}
        for k in toRemove:
            state_to_node.pop(k)
        vals = set(state_to_node.values())
        choreography.statements = vals

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
                substitution = { k : (trg if v == src else v)  for k, v in substitution.items() }
                substitution[src] = trg
        # now do the remove
        for s in removedStates:
            state_to_node.pop(s)
            choreography.start_state = substitution.get(choreography.start_state, choreography.start_state)
        for node in state_to_node.values():
            node.start_state = [ substitution.get(s, s) for s in node.start_state ]
            node.end_state = [ substitution.get(s, s) for s in node.end_state ]
            if isinstance(node, GuardedChoice):
                for g in node.guarded_states:
                    g.id = substitution.get(g.id, g.id)
        # remove unreachable node
        self.removeUnreachable(choreography, state_to_node)

    def flattenInternalChoice(self, choreography, state_to_node):
        # assumes no loop without any event
        for n in state_to_node.values():
            if isinstance(n, GuardedChoice):
                n.guarded_states = n.get_successors()
                n.end_state = [ g.id for g in n.guarded_states ]
        self.removeIndirections(choreography, state_to_node)

    # for the normalization
    def removeForkJoin(self, choreography, state_to_node, debug = True):
        # if there are nested forks then we should work inside out
        def noForkBeforeJoin(state, first = True):
            node = state_to_node[state]
            if isinstance(node, Fork) and not first:
                return False
            elif isinstance(node, Join):
                return True
            else:
                return all([noForkBeforeJoin(s, False) for s in node.end_state])
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
        def hasAction(state, first = True):
            node = state_to_node[state]
            if isinstance(node, Motion) and node.motions[0].mp_name != "wait":
                return True
            elif isinstance(node, SendMessage) or isinstance(node, GuardedChoice) or isinstance(node, ReceiveMessage):
                return True
            elif isinstance(node, Join):
                return first
            else:
                return any(hasAction(s, False) for s in node.end_state)
        # pull together the motion primitives
        # takes a list of threads and generate the permutation of events
        # TODO diverge in case of loop within fork-join
        def mergeThreads(pred, pred_index, pendingMotion, states):
            if debug:
                print("  pred: ", pred, " @ ", pred_index)
                print("  states: ", states)
                print("  pendingMotion: ", pendingMotion)
            states = [ s for s in states if hasAction(s) ]
            if debug:
                print("  relevant states: ", states)
            #first step categorise the next event
            internal = []
            external = []
            motion = []
            send = []
            receive = []
            join = []
            merge = []
            for s in states:
                node = state_to_node[s]
                if isinstance(node, GuardedChoice):
                    internal.append(s)
                elif isinstance(node, ExternalChoice):
                    external.append(s)
                elif isinstance(node, Motion):
                    motion.append(s)
                elif isinstance(node, SendMessage):
                    send.append(s)
                elif isinstance(node, ReceiveMessage):
                    receive.append(s)
                elif isinstance(node, Merge):
                    merge.append(s)
                elif isinstance(node, Join):
                    join.append(s)
                else:
                    raise Exception("mergeThreads: " + str(node))
            if len( { state_to_node[n].end_state[0] for n in join} ) > 1:
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
                internal = [newIntId]
                state_to_node[newIntId] = newInt
            # merge the external choices (temporary allow multiple motions in parallel)
            if len(external) + len(receive) + min(1, len(motion)) > 1: #all the motion are 1 choice (TO)
                newExtId = self.get_artificial_name()
                nodes = [ e for n in external for e in state_to_node[n].end_state ] + [ state_to_node[r].start_state[0] for r in receive ] + [ state_to_node[m].start_state[0] for m in motion ]
                newExt = ExternalChoice([newExtId], nodes)
                external = [newExtId]
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
                    assert len(join) == 0, "motion " + str(motion) + ", join " + str(join)
                    motion.extend(pendingMotion)
                    #TODO a bit too specialized
                    goodOnes = [ m for m in motion if state_to_node[m].motions[0].mp_name != "wait" ]
                    assert len(goodOnes) == 1, "no real motion: " + str(motion)
                    mp = copy(state_to_node[goodOnes.pop()])
                    newId = self.get_artificial_name()
                    mp.start_state[0] = newId
                    pred.end_state[pred_index] = newId
                    state_to_node[newId] = mp
                    successors = [ end for m in motion for end in state_to_node[m].end_state ]
                    return mergeThreads(mp, 0, [], successors)
                else:
                    assert len(pendingMotion) == 0
                    joinTarget = { state_to_node[n].end_state[0] for n in join}
                    assert len(joinTarget) == 1
                    return [(pred, pred_index)], joinTarget.pop()
            elif len(msgAlternatives) == 1:
                a = state_to_node[msgAlternatives.pop()]
                if debug:
                    print("    next (unique) action:", a)
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
                joinAt = None
                pendingMP = pendingMotion
                if isinstance(a2, ExternalChoice):
                    mpId = [ s for s in a2.end_state if isinstance(state_to_node[s], Motion) ]
                    if len(mpId) > 1:
                        reprMP = mpId.pop()
                        a2.end_state = [ s for s in a2.end_state if not isinstance(state_to_node[s], Motion) ].append(mpId)
                        pendingMP.extend(mpId)
                for i, e in enumerate(a2.end_state):
                    lst, j = mergeThreads(a2, i, pendingMP, motion + [e] + join)
                    acc.extend(lst)
                    if joinAt is None:
                        joinAt = j
                    else:
                        assert(joinAt == j)
                return acc, joinAt
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
                joinAt = None
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
                            pendingMP.extend(mpId)
                    # continue with the rest ...
                    for i, e in enumerate(a2.end_state):
                        # in case of timee, ignore the non-time events
                        if isinstance(a2, ExternalChoice) and isinstance(state_to_node[e], Motion):
                            remaining = [ mp for r in remaining for mp in findMp(r) ]
                        lst, j = mergeThreads(a2, i, pendingMP, remaining + motion + [e] + join)
                        acc.extend(lst)
                        if joinAt is None:
                            joinAt = j
                        else:
                            assert(joinAt == j)
                if debug:
                    print("    next choice action:", choice)
                return acc, joinAt
        # get the fork we need to remove
        forks = [ state for (state, node) in state_to_node.items() if isinstance(node, Fork) ]
        # the non-nested ones
        nonNestedFork = [ s for s in forks if noForkBeforeJoin(s) ]
        while len(forks) > 0:
            assert len(nonNestedFork) > 0
            for s in nonNestedFork:
                pred, idx = findPred(s)
                lastIds, join = mergeThreads(pred, idx, [], state_to_node[s].end_state)
                if debug:
                    print("    connecting", lastIds, "to", join)
                # replace the last join by a merge is needed
                if len(lastIds) == 1:
                    last, idx = lastIds.pop()
                    last.end_state[idx] = join
                else:
                    merge = Merge([], [join])
                    for pred, index in lastIds:
                        newId = sefl.get_artificial_name()
                        pred.end_state[index] = newId
                        merge.start_state.append(newId)
            # update work list
            if debug:
                print("after processing:", nonNestedFork)
                for v in state_to_node.values():
                    print(v)
            forks = [ s for s in forks if not s in nonNestedFork ]
            nonNestedFork = [ s for s in forks if noForkBeforeJoin(s) ]
            # remove unreachable node
            self.removeUnreachable(choreography, state_to_node)

    def removeMerge(self, choreography, state_to_node):
        # get rid of the merge
        removed = set()
        for node in state_to_node.values():
            if isinstance(node, Merge) and not node in removed:
                removed.add(node)
                post = node.end_state[0]
                for pre in state_to_node.values():
                    for i, p in enumerate(pre.end_state):
                        if p in node.start_state:
                            pre.end_state[i] = post
                            if isinstance(pre, GuardedChoice):
                                pre.guarded_states[i].id = post
        self.removeUnreachable(choreography, state_to_node)

    def minimize(self, choreography, state_to_node, debug = True):
        self.removeMerge(choreography, state_to_node)
        #states that can be similar
        compatible = {}
        allStates = { n.start_state[0] for n in state_to_node.values() }
        for s in allStates:
            compatible[s] = copy(allStates)
        #test similar
        def same(s1, s2):
            if s1 == s2:
                return True
            node1 = state_to_node[s1]
            node2 = state_to_node[s2]
            if type(node1) != type(node2) or len(node1.end_state) != len(node2.end_state):
                #print("not same(1)", node1, node2)
                return False
            #TODO should be up to permutation ...
            for i,e1 in enumerate(node1.end_state):
                if not node2.end_state[i] in compatible[e1]:
                    #print("not same(2)", node1, node2)
                    return False
            if isinstance(node1, GuardedChoice):
                return node1.guarded_states == node2.guarded_states
            elif isinstance(node1, SendMessage):
                return node1.receiver == node2.receiver and node1.msg_type == node2.msg_type and node1.expressions == node2.expressions
            elif isinstance(node1, ReceiveMessage):
                return node1.msg_type == node2.msg_type and node1.expressions == node2.expressions
            elif isinstance(node1, Motion):
                return node1.motions == node2.motions
            else:
                return True
        #pruning
        changed = True
        while changed:
            changed = False
            for s1 in allStates:
                print(s1, "==>", compatible[s1])
                for s2 in copy(compatible[s1]):
                    if not same(s1, s2):
                        changed = True
                        compatible[s1].discard(s2)
                        compatible[s2].discard(s1)
        if debug:
            print("equivalence classes")
            for n, eqs in compatible.items():
                print(n, "->", eqs)
        # representative for eq class
        mapsTo = {}
        def findRep(state):
            if state in mapsTo:
                return mapsTo[state]
            else:
                mapsTo[state] = state
                for s in compatible[state]:
                    assert s == state or not s in mapsTo, str(s) + str(mapsTo)
                    mapsTo[s] = state
                return state
        # reduce
        visited = set()
        choreography.start_state = findRep(choreography.start_state)
        frontier = { choreography.start_state }
        while len(frontier) > 1:
            state = frontier.pop()
            if not state in visited:
                visited.add(state)
                node = state_to_node[state]
                for i,e in end_state(node.end_state):
                    r = findRep(e)
                    frontier.add(r)
                    node.end_state[i] = r
                    if isinstance(node, GuardedChoice):
                        node.guarded_states[i].id = r
                if isinstance(node, GuardedChoice):
                    toRemove = {}
                    for i in range(0, len(node.guarded_states)):
                        for j in range(0, i):
                            if (not j in toRemove) and node.guarded_states[i].id == node.guarded_states[j].id:
                                node.guarded_states[j].expression = Or(node.guarded_states[j].expression, node.guarded_states[i].expression)
                                toRemove.add(i)
                                break
                    node.guarded_states = [ gs for i,gs in enumerate(node.guarded_states) if not i in toRemove ]
                    node.end_state = [gs.id for gs in node.guarded_states]
                else:
                    node.end_state = list(set(node.end_state))
        self.removeUnreachable(choreography, state_to_node)

    def checkProjection(self, choreography, state_to_node):
        def nextActionIsExternal(node):
            for n1 in node.end_state:
                node1 = state_to_node[n1]
                if isinstance(node1, ReceiveMessage) or isinstance(node1, Motion):
                    return True
                elif isinstance(node1, SendMessage) or isinstance(node1, GuardedChoice):
                    return False
                else:
                    return all(nextActionIsExternal(state_to_node[s]) for s in node1.end_state)
        for node in state_to_node.values():
            if isinstance(node, ExternalChoice):
                assert nextActionIsExternal(node), "external choice followed by internal action: " + str(node)
                for n1 in node.end_state:
                    node1 = state_to_node[n1]
                    for n2 in node.end_state:
                        if n1 < n2:
                            node2 = state_to_node[n2]
                            if isinstance(node1, Motion) and isinstance(node2, Motion):
                                raise Exception("ambiguous external choice for motion: " + str(node1) + ", " + str(node2))
                            if isinstance(node1, ReceiveMessage) and isinstance(node2, ReceiveMessage) and node1.msg_type == node2.msg_type:
                                raise Exception("ambiguous external choice for message: " + str(node1) + ", " + str(node2))


    def normalize_projection(self, choreography, state_to_node, debug = False):
        self.removeIndirections(choreography, state_to_node)
        if debug:
            print("-- after remove indirections --")
            print(choreography)
        self.removeForkJoin(choreography, state_to_node)
        if debug:
            print("-- after remove fork/join --")
            print(choreography)
        self.minimize(choreography, state_to_node)
        if debug:
            print("-- after minimize --")
            print(choreography)
        self.checkProjection(choreography, state_to_node)


