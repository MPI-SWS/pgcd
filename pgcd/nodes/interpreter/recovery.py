import verification.choreography.ast_chor as ast_chor
from verification.choreography.threads as ThreadChecks
from verification.choreography.synchronizability as SyncTracker
from verification.spec.time import DurationSpec
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
        self.names = dict()

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

    # getting fresh names
    def fresh(self, name):
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
                s2 = [ self.fresh(n) for n in s ]
                nodes.append(ast_chor.Fork([new_name],node.footprints,s2))
                ends = { traverse(n2, n) for (n2,n) in zip(s2,s) }
                ns = [ n for n in nodes if n.start_state[0] in ends ]
                for n in ns:
                    nodes.remove(n)
                joins = [n in ns if isinstance(n, ast_chor.Join)]
                join_name = 'join'
                if len(joins) > 0:
                    join_name = joins[0].end_state[0]
                new_join_name = self.fresh(join_name)
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
                n2 = self.fresh(n)
                nodes.append(ast_chor.GuardedChoice([new_name],[GuardArg(sp.true,n2)]))
                return traverse(n2, n)
            elif isinstance(node, ast_chor.Merge):
                n = node.end_state[0]
                n2 = self.fresh(n)
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
                n2 = self.fresh(n)
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
                n2 = self.fresh(n)
                nodes.append(ast_chor.Message([new_name],node.sender,node.receiver,node.msg_type,node.expressions,[n2]))
                return traverse(n2, n)
            else:
                # copy and rename
                assert(len(node.start_state) == 1 and len(node.end_state) == 1)
                node2 = copy(node)
                node2.start_state = [new_name]
                n = node.end_state[0]
                n2 = self.fresh(n)
                node2.end_state = [n2]
                return traverse(n2, n)
        first = self.fresh(chkpt)
        last = traverse(first, chkpt)
        c2 = ast_chor.Choreography("recovery_"+self.choreography.id, nodes, sp.true, first)
        c2.world = self.choreography.world
        return c2

    def stripPath(self, path):
        state_to_node = path.mk_state_to_node()
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
        # run the thread analysis to get the processes at each state
        tc = ThreadChecks(comp, comp.world)
        state_to_procs = { s:t.processes for (s,t) in tc.perform().items() }
        state_to_node = comp.mk_state_to_node()
        # extra info to synchronize
        state_to_duration = { comp.start_state:SyncTracker([DurationSpec(0,0,False)],comp.allProcesses()) }
        def timeToLastSync(node):
            # TODO return a maps from process to duration
            pass
        def needSync(node):
            # a node need to be synced if it is followed by a fork or a motion
            if node.start_state[0] == comp.start_state or \
               isinstance(node, ast_chor.Fork) or \
               isinstance(node, ast_chor.Motion):
                return True
            else:
                return False
        # modify the last motion executed by proc before state
        # FIXME this is a crude hack
        def modifyLastMotion(state, proc, addWait = -1, addIdle = False):
            assert proc in state_to_procs[state]
            node = state_to_node[state]
            if isinstance(node, ast_chor.Motion):
                for m in node.mostions:
                    if addWait > 0:
                        m.mp_name = m.mp_name + "_wait(" + str(addWait) + ")"
                    elif addIdle:
                        m.mp_name = m.mp_name + "_idle"
                assert False, "?!?"
            else:
                for pred in node.start_state:
                    if proc in state_to_procs[pred]:
                        return modifyLastMotion(pred, proc, addWait, addIdle)
                assert False, "!?!"
        # insert sync _before_ state
        # traverse like a topological sort
        to_process = copy(comp.statements)
        processed = set()
        frontier = { comp.start_state }
        while len(to_process) > 0:
            assert len(frontier) > 0
            state = frontier.pop()
            node = state_to_node[state]
            # START bookkeeping about what to process
            processed.add(state)
            to_process.remove(node)
            for s in node.end_state:
                assert not s in processed
                n = state_to_node[s]
                if all(pred in processed for pred in n.state_to_node):
                    frontier.add(s)
            # END bookkeeping
            if needSync(node):
                # get the duration for each process in the thread
                procs = state_to_procs[state]
                ds = timeToLastSync(node)
                # normalise the durations (make all uninteruptible)
                ds2 = dict()
                for (p,d) in ds.items():
                    if d.interruptible:
                        ds2[p] = DurationSpec(d.min,d.min,False)
                    else:
                        ds2[p] = d
                # find the best sender: minimize |d.max-d.min| and use name as tie breaker
                # FIXME not the optimal strategy but it will do
                receivers = list(procs).sort(key = lambda x: (ds2[x].max - ds2[x].min, d.name))
                sender = receivers.pop()
                # insert messages
                start_state = state
                for r in receivers:
                    end_state = start_state
                    start_state = fresh("sync")
                    msg = ast_chor.Message([start_state], sender, r, OK, [], [end_state])
                    comp.statements.append(msg)
                    # insert into aux lookup maps
                    state_to_node[start_state] = msg
                    state_to_procs[start_state] = state_to_procs[end_state]
                    state_to_duration[start_state] = state_to_duration[end_state].copy() # XXX
                if node.start_state[0] == comp.start_state:
                    comp.start_state = start_state
                else:
                    for n2 in comp.statements:  #find pred and rename
                        if n2.end_state[0] == node.start_state[0]:
                            n2.end_state = [start_state]
                # modifyLastMotion if needed
                max_duration = ds2[sender].max
                for r in receivers:
                    max_duration = max(max_duration, ds2[r].max)
                if ds2[sender].min < max_duration:
                    modifyLastMotion(state, sender, addWait = max_duration - ds2[sender].min)
                for r in receivers:
                    if ds2[r].max < max_duration:
                        modifyLastMotion(state, r, addIdle = True)

    def computeRecoveryChoreo(self, comp_info):
        chkpt = self.getCheckpoint(comp_info)
        path = self.getCheckpoint(comp_info, chkpt)
        self.stripPath(path)
        comp = self.reversePath(path)
        self.synchronize(comp)
        return comp #ready to be projected and run
