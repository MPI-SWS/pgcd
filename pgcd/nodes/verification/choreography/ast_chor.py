from enum import Enum
import sympy as sp
from spec.contract import *


class Type(Enum):
    statement = 1
    message = 2
    merge = 3
    join = 4
    fork = 5
    guard = 7
    join_fork_arg = 8
    expression = 10
    motion = 11
    end = 12
    send_message = 13
    receive_message = 14
    indirection = 15
    external_choice = 16
    checkpoint = 17


class Choreography():

    def __init__(self, id, statements, predicate, start_state):
        self.id = id
        self.statements = statements
        self.predicate = predicate
        self.start_state = start_state
        self.world = None # world is a spec.Component
        self.state_to_processes  = None # filled later by the analysis
        self.state_to_contracts  = None # filled later by the analysis

    def __str__(self):
        string = "def " + self.id + " \n"
        for stmt in self.statements:
            string += str(stmt) + '\n'
        string += " in [" + str(self.predicate) + ']' + str(self.start_state)
        return string

    def mk_state_to_node(self):
        state_to_node = {}
        for s in self.statements:
            for pre in s.start_state:
                assert not pre in state_to_node, str(pre) + " aleady in state_to_node"
                state_to_node[pre] = s
        return state_to_node

    def getProcessNames(self):
        procs = set()
        for s in self.statements:
            if isinstance(s, Message):
                procs.add(s.sender)
                procs.add(s.receiver)
            if isinstance(s, Motion):
                for m in s.motions:
                    procs.add(m.id)
        return procs

    def getProcess(self, name):
        if self.world == None:
            return None
        lst = [ p for p in self.world.allProcesses() if p.name() == name ]
        if len(lst) > 0:
            assert len(lst) == 1, "getProcess " + str(lst)
            return lst[0]
        else:
            return None

    def getProcessesNamesAt(self, n):
        if isinstance(n, type('  ')):
            name = n
        else: # a node
            name = n.start_state[0]
        return self.state_to_processes[name]

    def getProcessesAt(self, n):
        procs = self.getProcessesNamesAt(n)
        return { self.getProcess(p) for p in procs }

    def hasParallel(self):
        return any( isinstance(node, Fork) for node in self.statements )
    
    def getMotion(self, ProcessName, motionName, motionArgs):
        p = self.getProcess(process)
        return p.motionPrimitive(motionName, *motionArgs)


class DistributedStateNode():

    def __init__(self, tip, start_state, end_state):
        self.tip = tip
        self.start_state = start_state
        self.end_state = end_state

    def __str__(self):
        return Node.__str__(self) + ": " + str(self.start_state) + " -> " + str(self.end_state)


class Message(DistributedStateNode):

    def __init__(self, start_state, sender, receiver, msg_type, expressions, continue_state):
        DistributedStateNode.__init__(self, Type.message, start_state, continue_state)
        if sender == receiver:
            raise Exception("No self message! (" + sender + "->" + receiver + ")")
        self.sender = sender
        self.receiver = receiver
        self.msg_type = msg_type
        self.expressions = expressions

    def __str__(self):
        string = ''
        string += self.start_state[0] + '=' + self.sender + '->' + self.receiver + ':' + self.msg_type + '('
        for x in self.expressions:
            string += str(x)
        string += ');' + self.end_state[0]
        return string

    def accept(self, visitor):
        visitor.visit(self)


class Motion(DistributedStateNode):

    def __init__(self, start_state, motions, end_state):
        DistributedStateNode.__init__(self, Type.motion, start_state, end_state)
        self.motions = motions

    def __str__(self):
        string = ''.join(self.start_state) + ' = ('
        for x in self.motions:
            string += str(x)
            if x != self.motions[-1]:
                string += ','
        string += '); ' + ''.join(self.end_state)
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        for mot1, mot2 in zip(self.motions, node.motions):
            if not mot1.shift_delay_check(mot2):
                return False
        return True

class MotionArg():

    def __init__(self, id, mp_name, mp_args):
        self.id = id
        self.mp_name = mp_name
        self.mp_args = mp_args

    def __str__(self):
        string = str(self.id) + ': ' + self.mp_name + "("
        string += ", ".join([str(a) for a in self.mp_args]) + ')'
        return string

    def shift_delay_check(self, node):
        return self.id == node.id and self.mp_name == node.mp_name and self.mp_args == node.mp_args


class GuardedChoice(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.guard, start_state, [x.id for x in continue_state])
        self.guarded_states = continue_state

    def __str__(self):
        string = ''.join(self.start_state) + ' = '
        for x in self.guarded_states:
            string += str(x)
            if x != self.guarded_states[-1]:
                string += ' + '
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        for g1, g2 in zip(self.guarded_states, node.guarded_states):
            if not g1.shift_delay_check(g2):
                return False
        return True

    def get_successors(self, state_to_node):
        '''get first non guarded successors (accumulate the guards)'''
        acc = []
        for g in self.guarded_states:
            node = state_to_node[g.id]
            if isinstance(node, GuardedChoice):
                successors = node.get_successors(state_to_node)
                for s in successors:
                    new_guard = GuardArg(sp.And(g.expression, s.expression), s.id)
                    acc.append(new_guard)
            else:
                acc.append(g)
        return acc

class GuardArg():

    def __init__(self, expression, id):
        self.id = id
        self.expression = expression
        self.has_motion = None # for normalization projection

    def __str__(self):
        string = '['
        string += str(self.expression)
        string += '] ' + str(self.id)
        return string

    def shift_delay_check(self, node):
        return self.id == node.id and self.expression == node.expression


class Merge(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.merge, start_state, continue_state)

    def __str__(self):
        string = ''
        for x in self.start_state:
            string += str(x)
            if x != self.start_state[-1]:
                string += ' + '
        string += ' = ' + ''.join(self.end_state)
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return  self == node

# TODO contract rather than just footprints
class Fork(DistributedStateNode):

    def __init__(self, start_state, footprints, continue_state):
        DistributedStateNode.__init__(self, Type.fork, start_state, continue_state)
        self.footprints = footprints

    def __str__(self):
        string = ''.join(self.start_state) + ' = '
        for fp, x in zip(self.footprints, self.end_state):
            if fp != None:
                string += str(fp)
                string += ' '
            string += str(x)
            if x != self.end_state[-1]:
                string += ' || '
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return  self == node

class Footprint():

    def __init__(self, x, y, z, expr):
        self.x = x
        self.y = y
        self.z = z
        self.expr = expr

    def toFpContract(self, start_state, processes, frame):
        return FpContract(start_state + " fp contract", processes, frame, self.x, self.y, self.z, self.expr) 

    def __str__(self):
        return "{ (" + str(self.x) + ", " +  str(self.y) + ", " + str(self.z) + ") : " + str(self.expr) + " }"

class Join(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.join, start_state, continue_state)
        self.check_states = []

    def __str__(self):
        string = ''
        for x in self.start_state:
            string += str(x)
            if x != self.start_state[-1]:
                string += '||'
        string += ' = ' + ''.join(self.end_state)
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return  self == node


class End(DistributedStateNode):

    def __init__(self, start_state):
        DistributedStateNode.__init__(self, Type.end, start_state, [])

    def __str__(self):
        return ''.join(self.start_state) + ' = end'

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return  self == node


class Checkpoint(DistributedStateNode):

    def __init__(self, start_state, end_state, id):
        DistributedStateNode.__init__(self, Type.checkpoint, start_state, end_state)
        self.id = id

    def __str__(self):
        return ''.join(self.start_state) + ' = checkpoint('+str(self.id)+').' + ''.join(self.end_state)

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return  self == node
