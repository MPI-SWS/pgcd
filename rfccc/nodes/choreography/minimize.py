from copy import copy
from parser_chor import *
from ast_chor import *
from ast_proj import *

def removeUnreachable(choreography, state_to_node):
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
    
def removeIndirections(choreography, state_to_node):
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
    removeUnreachable(choreography, state_to_node)
    
def removeMerge(choreography, state_to_node):
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
    removeUnreachable(choreography, state_to_node)

def minimize(choreography, state_to_node, debug = False):
    removeMerge(choreography, state_to_node)
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
            return False
        #TODO should be up to permutation ...
        for i,e1 in enumerate(node1.end_state):
            if not node2.end_state[i] in compatible[e1]:
                #print("not same(2)", node1, node2)
                return False
        if isinstance(node1, GuardedChoice):
            return node1.guarded_states == node2.guarded_states
        elif isinstance(node1, Message):
            return node1.sender == node2.sender and node1.receiver == node2.receiver and node1.msg_type == node2.msg_type and node1.expressions == node2.expressions
        elif isinstance(node1, SendMessage):
            return node1.receiver == node2.receiver and node1.msg_type == node2.msg_type and node1.expressions == node2.expressions
        elif isinstance(node1, ReceiveMessage):
            return node1.msg_type == node2.msg_type and node1.expressions == node2.expressions
        elif isinstance(node1, Motion):
            for i, m1 in enumerate(node1.motions):
                m2 = node2.motions[i]
                if not m1.shift_delay_check(m2):
                    return False
            return True
        else:
            return True
    #pruning
    changed = True
    while changed:
        changed = False
        for s1 in allStates:
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
    while len(frontier) > 0:
        state = frontier.pop()
        if not state in visited:
            visited.add(state)
            node = state_to_node[state]
            for i,e in enumerate(node.end_state):
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
    removeUnreachable(choreography, state_to_node)
