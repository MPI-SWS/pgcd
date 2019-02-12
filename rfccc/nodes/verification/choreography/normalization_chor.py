from copy import copy
from sympy import S
from minimize import *
from parser_chor import *
from ast_chor import *
from ast_proj import *

def removeForkJoin(nameGen, choreography, state_to_node, debug = False):
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
        if isinstance(node, Motion) and not isPlaceholderMp(node.motions[0]):
            return True
        elif isinstance(node, Message) or isinstance(node, GuardedChoice):
            return True
        elif isinstance(node, Join):
            return first
        else:
            return any(hasAction(s, False) for s in node.end_state)
    # pull together the motion primitives
    # takes a list of threads and generate the permutation of events
    # TODO diverge in case of loop within fork-join
    def mergeThreads(pred, pred_index, states):
        if debug:
            print("  pred: ", pred, " @ ", pred_index)
            print("  states: ", states)
        states = [ s for s in states if hasAction(s) ]
        if debug:
            print("  relevant states: ", states)
        #first step categorise the next event
        choice = []
        message = []
        motion = []
        join = []
        merge = []
        for s in states:
            node = state_to_node[s]
            if isinstance(node, GuardedChoice):
                choice.append(s)
            elif isinstance(node, Motion):
                motion.append(s)
            elif isinstance(node, Message):
                message.append(s)
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
            states = choice + motion + message + merged + join
            return mergeThreads(pred, pred_index, states)
        if len(choice) + len(message) > 1: #still something to do without spending time
            # new choice
            newId = nameGen.get_artificial_name()
            choice = GuardedChoice([newId], [GuardArg(S.true(), newId) for x in (choice+message)])
            state_to_node[newId] = choice
            #update the predecessor
            pred.end_state[pred_index] = newId
            if isinstance(pred, GuardedChoice):
                pred.guarded_states[pred_index].id = newId
            # one action
            acc = []
            joinAt = None
            for i, a in (choice+message):
                remaining = copy((choice+message)).pop(i)
                a2 = copy(state_to_node[a])
                newId2 = nameGen.get_artificial_name()
                a2.start_state[0] = newId2
                state_to_node[newId2] = a2
                choice.end_state[i] = newId2
                choice.guarded_states[i].id = newId2
                # continue with the rest ...
                for i, e in enumerate(a2.end_state):
                    lst, j = mergeThreads(a2, i, remaining + motion + [e] + join)
                    acc.extend(lst)
                    if joinAt is None:
                        joinAt = j
                    else:
                        assert(joinAt == j)
            if debug:
                print("    next choice action:", choice)
            return acc, joinAt
        elif len(choice) + len(message) == 1:
            a = state_to_node[(choice+message).pop()]
            if debug:
                print("    next (unique) action:", a)
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
            for i, e in enumerate(a2.end_state):
                lst, j = mergeThreads(a2, i, motion + [e] + join)
                acc.extend(lst)
                if joinAt is None:
                    joinAt = j
                else:
                    assert(joinAt == j)
            return acc, joinAt
        elif len(motion) >= 1: # motions
            assert len(join) == 0, "motion " + str(motion) + ", join " + str(join)
            mps = [ m for ms in motion for m in state_to_node[ms].motions ] 
            newId = nameGen.get_artificial_name()
            newDest = nameGen.get_artificial_name() # just a placeholder
            mp = Motion([newId], mps, [newDest]) 
            pred.end_state[pred_index] = newId
            state_to_node[newId] = mp
            successors = [ end for m in motion for end in state_to_node[m].end_state ]
            return mergeThreads(mp, 0, successors)
        elif len(join) >= 1:
            head = join.pop()
            # only joins
            assert( all( state_to_node[j] == state_to_node[head] for j in join ) ), [head] + join
            return [(pred, pred_index)], state_to_node[head].end_state[0]
        else:
            raise Exception("nothing to do !?")
    # get the fork we need to remove
    forks = [ state for (state, node) in state_to_node.items() if isinstance(node, Fork) ]
    # the non-nested ones
    nonNestedFork = [ s for s in forks if noForkBeforeJoin(s) ]
    while len(forks) > 0:
        assert len(nonNestedFork) > 0
        for s in nonNestedFork:
            pred, idx = findPred(s)
            lastIds, join = mergeThreads(pred, idx, state_to_node[s].end_state)
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
        choreography.statements = state_to_node.values
        removeUnreachable(choreography, state_to_node)
        forks = [ s for s in forks if not s in nonNestedFork ]
        nonNestedFork = [ s for s in forks if noForkBeforeJoin(s) ]
