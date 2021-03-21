import verification.choreography.ast_chor as ast_chor
import sympy as sp
import logging
from copy import copy

log = logging.getLogger("Recovery")

# TODO implement a service to deal with motion failure and recover from it
# all the communication about error goes in here to avoid clutering the interpreter
# needs to know:
#   the participants (how many messages to receive)

class Recovery:

    def __init__(self, interpreter, choreography):
        self.interpreter = interpreter
        self.choreography = choreography
        self.error_topic = pass #TODO
        self.compensation_topic = pass #TODO

    def start(self, robots):
        # start a thread to listen to the error topic
        # after the error topic, is should listen to the compensation_topic
        pass

    def stop(self):
        pass

    def failure(self):
        message = error_message() # TODO
        message.header.frame_id = self.interpreter.id
        # message.header.stamp = rclpy.Time.now()
        pub = self.error_topic
        pub.publish(message)

    def send_comp(self):
        #TODO
        # - get the history from the robot
        # - apply inverse
        # - serialize
        # - send
        # the compensation info are
        # - checkpoint with (-,ID list)
        # - message with (msg type, msg args)
        # - motions with (name, args)
        pass

    def wait_for_comp_info(self):
        #TODO gather the compensations from all the robots
        pass

    def getCheckpoint(self, comp_info):
        # bottom element is a checkpoint get the ids
        chkpts = [ set(s[0][2]) for (p,s) in comp_info.items ]
        chkpt = chkpts.pop()
        for pt in chkpts:
            chkpt &= pt
        assert len(chkpt) == 1
        c = chkpt.pop()
        for node in self.choreography.statements:
            if isinstance(node, ast_chor.Checkpoint) and node.id == c:
                return node.end_state[0]
        assert False, "checkpoint " + str(c) + " not found."

    def getPath(self, comp_info, chkpt)
        procs = self.choreography.allProcesses()
        state_to_node = self.choreography.mk_state_to_node()
        stacks = dict()
        for p in procs:
            stacks[p] = comp_info[p][1:] # remove the checkpoint
        # getting fresh names
        names = dict()
        def fresh(name):
            if name in names:
                i = names[name]
                names[name] = i+1
                return name + "_" + str(i)
            else:
                names[name] = 1
                return name + "_0"
        def last(name):
            if name in names:
                return name + "_" + str(names[name]-1)
            else:
                return name + "_0"
        # lookahead to figure out which branch was taken (1st message sent)
        def isBranchPossible(self, state):
            node = state_to_node(state)
            if isinstance(node, ast_chor.Message):
                stack = stacks[node.sender]
                action = stack[0]
                return action[0] == ActionType.MESSAGE and action[1] == node.msg_type
            else:
                assert False, 'expected message after ' + state
        # return a choice less choreography
        nodes = [] # accumulate nodes for the new choreo
        def traverse(new_name, state):
            node = state_to_node(state)
            if isinstance(node, ast_chor.Fork):
                s = node.end_state
                s2 = [ fresh(n) for n in s ]
                nodes.append(ast_chor.Fork([new_name],node.footprints,s2))
                ends = { traverse(n2, n) for (n2,n) in zip(s2,s) }
                ns = [ n for n in nodes if n.start_state[0] in ends ]
                for n in ns:
                    nodes.remove(n)
                joins = [n in ns if isinstance(n, ast_chor.Join)]
                join_name = 'join'
                if len(joins) > 0:
                    join_name = joins[0].end_state[0]
                new_join_name = fresh(join_name)
                nodes.append(ast_chor.Join(ends,[new_join_name]))
                if len(joins) == len(ends):
                    return traverse(new_join_name, join_name)
                else:
                    # a node has finished so everybody ends
                    nodes.append(ast_chor.End([new_join_name]))
                    return new_join_name
            elif isinstance(node, ast_chor.Join):
                nodes.append(ast_chor.Join([new_name],node.end_state))
                return new_name
            elif isinstance(node, ast_chor.GuardedChoice):
                possible = [ s for s in node.end_state if isBranchPossible(s) ]
                assert(len(possible) >= 0)
                if len(possible) > 1:
                    log.warning("more than 1 possible branch")
                n = possible[0]
                n2 = fresh(n)
                nodes.append(ast_chor.GuardedChoice([new_name],[GuardArg(sp.true,n2)]))
                return traverse(n2, n)
            elif isinstance(node, ast_chor.Merge):
                n = node.end_state[0]
                n2 = fresh(n)
                nodes.append(ast_chor.Merge([new_name],[n2]))
                return traverse(n2, n)
            elif isinstance(node, ast_chor.Motion):
                # consume the motions
                ms = []
                done = False
                for m in node.motions:
                    stack = stacks[m.id]
                    assert (not done) or len(stack == 1) # if one is done then all are done
                    done = len(stack == 1)
                    action = stack[0]
                    assert(action[0] == ActionType.MOTION)
                    ms.append(ast_chor.MotionArg(m.id, action[1], action[2]))
                    stacks[m.id] = stack[1:]
                n = node.end_state[0]
                n2 = fresh(n)
                nodes.append(ast_chor.Motion([new_name],ms,[n2]))
                # if done then insert "End"
                if done:
                    nodes.append(ast_chor.End([n2]))
                    return n2
                else:
                    return traverse(n2, n)
            elif isinstance(node, ast_chor.Message):
                # consume the message
                stack = stacks[node.sender]
                action = stack[0]
                assert(action[0] == ActionType.MESSAGE and action[1] == node.msg_type)
                stacks[node.sender] = stack[1:]
                n = node.end_state[0]
                n2 = fresh(n)
                nodes.append(ast_chor.Message([new_name],node.sender,node.receiver,node.msg_type,node.expressions,[n2]))
                return traverse(n2, n)
            else:
                # copy and rename
                assert(len(node.start_state) == 1 and len(node.end_state) == 1)
                node2 = copy(node)
                node2.start_state = [new_name]
                n = node.end_state[0]
                n2 = fresh(n)
                node2.end_state = [n2]
                return traverse(n2, n)
        first = fresh(chkpt)
        last = traverse(first, chkpt)
        c2 = ast_chor.Choreography("recovery_"+self.choreography.id, nodes, sp.true, first)
        c2.world = self.choreography.world
        return c2

    def stripPath(self, path):
        start_state = path.mk_state_to_node()
        toRemove = []
        for node in path.statements:
            # keep only Fork, Join, Motion, and End
            if isinstance(node, ast_chor.Fork) or \
               isinstance(node, ast_chor.Join) or \
               isinstance(node, ast_chor.Motion) or \
               isinstance(node, ast_chor.End):
                pass
            else:
                toRemove.append(node)
                assert len(node.end_state) == 1
                new_state = node.end_state[0]
                for state in node.start_state:
                    n2 = state_to_node[state]
                    end_state2 = [ s if s != state else new_state for s in n2.end_state ]
                    n2.end_state = end_state2
                if path.start_state == state:
                    path.start_state = new_state
        for n in toRemove
            path.statements.remove(n)

    def reversePath(self,path):
        start_state = None
        end_state = ast_chor.End([path.start_state])
        new_nodes = [end_state]
        for node in path.statements:
            if isinstance(node, ast_chor.Fork):
                new_nodes.append(ast_chor.Join(node.end_state,node.start_state))
            elif isinstance(node, ast_chor.Join):
                new_nodes.append(ast_chor.Fork(node.end_state,node.start_state))
            elif isinstance(node, ast_chor.Motion):
                new_nodes.append(ast_chor.Motion(node.end_state,node.motions,node.start_state))
            elif isinstance(node, ast_chor.End):
                start_state = node.start_state[0]
            else:
                assert False, "not expecting " + str(node)
        assert start_state != None
        c2 = ast_chor.Choreography("recovery_"+self.choreography.id, new_nodes, sp.true, start_state)
        c2.world = self.choreography.world
        return c2

    def synchronize(self, comp):
        # TODO
        # get a process order
        pass

    def computeRecoveryChoreo(self, comp_info):
        chkpt = self.getCheckpoint(comp_info)
        path = self.getCheckpoint(comp_info, chkpt)
        self.stripPath(path)
        unsynced = self.reversePath(path)
        synced = self.synchronize(unsynced)
        return synced

# find which path was taken in the recovery
# - intersection of all checkpoints IDs should be a singleton
# - follows the messages to get the path and find the fork/joins
# - compute a synchronization of the compensations (be deterministic)
