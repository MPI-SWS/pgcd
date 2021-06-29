from copy import copy
from sympy import S
from verification.choreography.minimize import *
from verification.choreography.parser_chor import *
from verification.choreography.ast_chor import *
from verification.choreography.ast_proj import *
import logging

log = logging.getLogger("Normalization")

def removeForkJoin(nameGen, choreography, state_to_node):
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
        if isinstance(node, Motion):
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
        log.debug("  pred: %s @ %s", pred, pred_index)
        log.debug("  states: %s", states)
        log.debug("  pendingMotion: %s", pendingMotion)
        states = [ s for s in states if hasAction(s) ]
        log.debug("  relevant states: %s", states)
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
            #FIXME this does not look right. it should be the combinations ?!? (2ⁿ)
            newIntId = nameGen.get_artificial_name()
            guards = [ g for i in internal for g in i.get_successors(state_to_node) ]
            newInt = GuardedChoice([newInt], guards)
            internal = [newIntId]
            state_to_node[newIntId] = newInt
        # merge the external choices (temporary allow multiple motions in parallel)
        if len(external) + len(receive) + min(1, len(motion)) > 1: #all the motion are 1 choice (TO)
            newExtId = nameGen.get_artificial_name()
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
                goodOnes = [ m for m in motion if not isPlaceholderMp(state_to_node[m].motions[0]) ]
                assert len(goodOnes) == 1, "no real motion: " + str(motion)
                mp = copy(state_to_node[goodOnes.pop()])
                newId = nameGen.get_artificial_name()
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
            log.debug("    next (unique) action: %s", a)
            a2 = copy(a)
            newId = nameGen.get_artificial_name()
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
            newId = nameGen.get_artificial_name()
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
                newId2 = nameGen.get_artificial_name()
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
            log.debug("    next choice action: %s", choice)
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
            log.debug("    connecting %s to %s", lastIds, join)
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
        if log.isEnabledFor(logging.DEBUG):
            log.debug("after processing: %s", nonNestedFork)
            for v in state_to_node.values():
                log.debug("%s", v)
        forks = [ s for s in forks if not s in nonNestedFork ]
        nonNestedFork = [ s for s in forks if noForkBeforeJoin(s) ]
        # remove unreachable node
        removeUnreachable(choreography, state_to_node)

#to make our life simpler, let us add an external choice before every receive
def addExternalChoice(nameGen, choreography, state_to_node):
    recv = [ node.start_state[0] for node in choreography.statements if isinstance(node, ReceiveMessage) ]
    needChoice = [ node for node in choreography.statements if not isinstance(node, ExternalChoice) and any(n in recv for n in node.end_state) ]
    for node in needChoice:
        for i, succ in enumerate(node.end_state):
            if succ in recv:
                name = nameGen.get_artificial_name()
                newExt = ExternalChoice([name], [succ])
                state_to_node[name] = newExt
                choreography.statements.add(newExt)
                node.end_state[i] = name
                if isinstance(node, GuardedChoice):
                    node.guarded_states[i].id = name
