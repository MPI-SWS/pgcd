from ast_chor import *
from sympy import *
from spec import *
from copy import *

def isPlaceholderMp(mp):
    return mp.mp_name == "__wait__"

def mkPlaceholderMp(process, duration):
    return MotionArg(process.name(), "__wait__", [duration])


def CreateProjectionFromChoreography(choreography, projection_name, process):
    ''' Creates a projection of a choreography on process '''
    pred = True
    for c in And.make_args(choreography.predicate):
        if set(c.free_symbols) <=  set(process.variables()):
            pred = And(pred, c)
    chor_proj = ChoreographyProjection(projection_name, [], pred, choreography.start_state, process.name())

    for node in choreography.statements:

        if isinstance(node, Message):
            if node.sender == process.name():
                node2 = SendMessage(node.start_state, node.receiver, node.msg_type, node.expressions, node.end_state)
                chor_proj.statements.append(node2)
            elif node.receiver == process.name():
                node2 = ReceiveMessage(node.start_state, node.msg_type, node.expressions, node.end_state)
                chor_proj.statements.append(node2)
            else:
                chor_proj.statements.append(Indirection(node.start_state, node.end_state))

        elif isinstance(node, Motion):
            motions = []
            for x in node.motions:
                if x.id == process.name():
                    motions.append(x)
            if motions == []:
                motions.append(mkPlaceholderMp(process, sympify(1))) # TODO DZ: fix the duration later
            n2 = Motion(node.start_state, motions, node.end_state)
            chor_proj.statements.append(n2)

        elif isinstance(node, GuardedChoice):
            if all([set(g.expression.free_symbols) <= set(process.variables()) for g in node.guarded_states]):
                chor_proj.statements.append(node)
            else:
                chor_proj.statements.append(ExternalChoice(node.start_state, [x.id for x in node.guarded_states]))

        else:
            chor_proj.statements.append(copy(node))

    return chor_proj


class ChoreographyProjection():

    def __init__(self, id, statements, predicate, start_state, process):
        self.id = id
        self.statements = statements
        self.predicate = predicate
        self.start_state = start_state
        self.process = process

    def __str__(self):
        string = "PROCESS: " + self.process + "\ndef " + self.id + "\n"
        for stmt in self.statements:
            string += str(stmt) + '\n'
        string += " in [" + str(self.predicate) + ']' + str(self.start_state)
        return string

    def mk_state_to_node(self):
        state_to_node = {}
        for s in self.statements:
            for pre in s.start_state:
                assert not pre in state_to_node
                state_to_node[pre] = s
        return state_to_node

class SendMessage(DistributedStateNode):

    def __init__(self, start_state, receiver, msg_type, exps, continue_state):
        DistributedStateNode.__init__(self, Type.send_message, start_state, continue_state)
        self.receiver = receiver
        self.msg_type = msg_type
        self.expressions = exps

    def __str__(self):
        string = self.start_state[0] + ' = ' + self.receiver + '!' + self.msg_type + '('
        for x in self.expressions:
            string += str(x)
        string += '); ' + self.end_state[0]
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return self.receiver == node.receiver and self.msg_type == node.msg_type and self.expressions == node.expressions


class ReceiveMessage(DistributedStateNode):

    def __init__(self, start_state, msg_type, expressions, continue_state):
        DistributedStateNode.__init__(self, Type.receive_message, start_state, continue_state)
        self.msg_type = msg_type
        self.expressions = expressions

    def __str__(self):
        string = self.start_state[0] + ' = ?' + self.msg_type + '('
        string += ', '.join([str(x) for x in self.expressions])
        string += '); ' + self.end_state[0]
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return self.msg_type == node.msg_type and self.expressions == node.expressions



class Indirection(DistributedStateNode):

    def __init__(self, start_state, end_state):
        DistributedStateNode.__init__(self, Type.indirection, start_state, end_state)

    def __str__(self):
        return self.start_state[0] + ' = ' + self.end_state[0]

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return  self == node


class ExternalChoice(DistributedStateNode):

    def __init__(self, start_state, end_state):
        DistributedStateNode.__init__(self, Type.external_choice, start_state, end_state)

    def __str__(self):
        string = ''.join(self.start_state) + ' = '
        for x in self.end_state:
            string += str(x)
            if x != self.end_state[-1]:
                string += ' & '
        return string

    def accept(self, visitor):
        visitor.visit(self)

    def shift_delay_check(self, node):
        return self == node

    def merge(self, new_start_state, nodes):
        assert(all([isinstance(node, ExternalChoice) for node in nodes]))
        new_node = ExternalChoice([new_start_state], self.end_state + node.end_state)
