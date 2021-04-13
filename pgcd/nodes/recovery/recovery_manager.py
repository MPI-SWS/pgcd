from interpreter.status import *
import verification.choreography.ast_chor as ast_chor
from recovery.motion_annot import *
from recovery.partial_program import *
import pgcd.msg
import sympy as sp
import logging
import threading
import time
from copy import copy

log = logging.getLogger("Recovery")

# TODO implement a service to deal with motion failure and recover from it
# all the communication about error goes in here to avoid clutering the interpreter
# needs to know:
#   the participants (how many messages to receive)

class RecoveryManager:

    def __init__(self, interpreter, choreography):
        self.interpreter = interpreter
        self.choreography = choreography
        self.names = dict()
        self.nProcesses = len(self.choreography.allProcesses())
        self.comps = dict()
        self.gotInfo = threading.Event()
        self.error_pub = None # filled by the Component
        self.compensation_pub = None # filled by the Component
        self.restartFrom = None
        self.isRecovering = False
        self.originalProgram = None
        self.lock = threading.Lock()

    def failure_callback(self, msg):
        log.warning("received error")
        self.interpreter.status = InterpreterStatus.INTERRUPTED #TODO sync

    def comp_callback(self, msg):
        (process, stack) = self.parseComp(msg)
        with self.lock:
            self.comps[process] = stack
            if len(self.comps) == self.nProcesses:
                # we are ready to start the recovery
                self.gotInfo.set()

    def startRecovery(self):
        self.sendComp()
        self.gotInfo.wait()
        (chkpt, rec_choreo) = self.computeRecoveryChoreo(self.comp_info)
        proc = rec_choreo.getProcess(self.interpreter.id)
        p = Proj2Code(c, proc)
        rec_code = p.getCode()
        # clean-up
        self.gotInfo.clear()
        self.comps.clear()
        # put the result in the interperter
        self.restartFrom = chkpt
        self.originalProgram = self.interperter.program
        self.interperter.program = rec_code
        self.interperter.status = InterpreterStatus.IDLE
        self.isRecovering = True
        # start (delay the 1st sender)
        if rec_code.children[0].isSend:
            time.sleep(1)

    def restoreProgram(self):
        assert self.isRecovering
        sefl.isRecovering == False
        new_prog = resumeAt(self.originalProgram, self.restartFrom)
        self.interperter.program = new_prog
        self.interperter.status = InterpreterStatus.IDLE

    def failure(self):
        message = pgcd.msg.ErrorStamped()
        message.header.frame_id = self.interpreter.id
        # message.header.stamp = rclpy.Time.now()
        self.error_pub.publish(message)

    def prepareCompEntry(self, entry):
        e = pgcd.msg.CompensationEntry()
        if entry[0] == ActionType.MESSAGE:
            e.kind = 1
            e.description = entry[1]
            e.parameters = entry[2]
        elif entry[0] == ActionType.MOTION:
            motion, args = self.interpreter.robot.inverse(entry[1], entry[2])
            e.kind = 2
            e.description = motion
            e.parameters = args
        else:
            assert False, "unexpected entry: " + str(entry)
        return e

    def prepareComp(self):
        msg = pgcd.msg.CompensationStamped()
        msg.header.frame_id = self.interpreter.id
        checkPt = self.interpreter.stack[0]
        assert checkPt[0] == ActionType.CHECKPOINT
        msg.compensation.checkpoint = checkPt[2]
        msg.compensation.entries = [ self.prepareCompEntry(e) for e in self.interpreter.stack[1:] ]
        return msg

    def sendComp(self):
        comp = self.prepareComp()
        self.compensation_topic.publish(comp)

    def parseCompEntry(self, entry):
        if entry.kind == 1:
            return (ActionType.MESSAGE, entry.description, entry.parameters)
        elif entry.kind == 2:
            return (ActionType.MOTION, entry.description, entry.parameters)
        else:
            assert False, "unexpected entry: " + str(entry)

    def parseComp(self, msg):
        process = msg.header.frame_id
        c = msg.compensation
        checkpts = set(c.checkpoint)
        stack = [ self.parseCompEntry(e) for e in c.entries ]
        stack.insert(0, (ActionType.CHECKPOINT, None, checkpts))
        return (process, stack)

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

    def getPath(self, comp_info, chkpt):
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
                joins = [n for n in ns if isinstance(n, ast_chor.Join)]
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
        for n in toRemove:
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

    def computeRecoveryChoreo(self, comp_info):
        chkpt = self.getCheckpoint(comp_info)
        path = self.getCheckpoint(comp_info, chkpt)
        self.stripPath(path)
        comp = self.reversePath(path)
        s = Synchronizer(comp)
        s.synchronize()
        return (chkpt, comp) #ready to be projected and run
