from ast_chor import *
from sympy import *
from spec import *



def CreateProjectionFromChoreography(choreography, projection_name, process):
    ''' Creates a projection of a choreography on process '''
    chor_proj = ChoreographyProjection(projection_name, [], choreography.start_state, choreography.end_state,
                                       process.name())
    state_to_node = {}

    for node in choreography.statements:

        if isinstance(node, Message):
            if node.comp1 == process.name():
                chor_proj.statements.append(
                    SendMessage(node.start_state, node.comp2, node.msg_type, node.expressions, node.end_state))
            elif node.comp2 == process.name():
                chor_proj.statements.append(
                    ReceiveMessage(node.start_state, node.msg_type, node.expressions, node.end_state))
            else:
                chor_proj.statements.append(Indirection(node.start_state, node.end_state))

        elif isinstance(node, Motion):
            motions = []
            for x in node.motions:
                if x.id == process.name():
                    chor_proj.statements.append(node)
                    continue
                motions.append(x.mp_name)
            chor_proj.statements.append(
                Motion(node.start_state, [MotionArg(process, "wait", sympify(1))],  # TODO DZ: fix that later
                       node.end_state))

        elif isinstance(node, GuardedChoice):
            if set(node.guarded_states.expression.free_symbols) | \
                    set(node.guarded_states.expression.atoms(Function)) <= process.ownVariables():
                chor_proj.statements.append(node)
            else:
                chor_proj.statements.append(ExternalChoice(node.start_state, [x.id for x in node.end_state]))
        else:
            chor_proj.statements.append(node)

        last = chor_proj.statements[-1]
        for state in last.start_state:
            state_to_node[state] = last

    return chor_proj, state_to_node


class ChoreographyProjection(DistributedStateNode):

    def __init__(self, id, statements, predicate, start_state, process):
        DistributedStateNode.__init__(self, Type.projection, start_state, None)
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

    def accept(self, visitor):
        visitor.visit(self)


class SendMessage(DistributedStateNode):

    def __init__(self, start_state, receiver, msg_type, exps, continue_state):
        DistributedStateNode.__init__(self, Type.send_message, start_state, continue_state)
        self.receiver = receiver
        self.msg_type = msg_type
        self.expressions = exps

    def __str__(self):
        string = 'Message'
        string += self.start_state[0] + '=' + self.receiver + '!' + self.msg_type + '('
        for x in self.expressions:
            string += str(x)
        string += ');' + self.end_state[0]
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
        string = 'Message'
        string += self.start_state[0] + '= ?' + self.msg_type + '('
        for x in self.expressions:
            string += str(x)
        string += ');' + self.end_state[0]
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
        string = ''.join(self.start_state) + '='
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
