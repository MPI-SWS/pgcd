from ast_chor import *
from sympy import *


def CreateProjectionFromChoreography(choreography, _id, process):
    tree = ChoreographyProjection(_id, [], choreography.start_state, choreography.end_state, process)

    for node in choreography.statements:

        if isinstance(node, Message):
            if node.comp1 == process:
                tree.statements.append(
                    SendMessage(node.start_state, node.comp2, node.msg_type, node.expressions, node.end_state))
            elif node.comp2 == process:
                tree.statements.append(ReceiveMessage(node.start_state, node.msg_type, node.expressions, node.end_state))
            else:
                tree.statements.append(Indirection(node.start_state, node.end_state))

        elif isinstance(node, Motion):
            motions = []
            for x in node.motions:
                if x.id == process:
                    tree.statements.append(node)
                    continue
                motions.append(str(x.sympy_formula).split('(')[0])
            tree.statements.append(
                Motion(node.start_state, [MotionArg(process, sympify('"wait(duration(' + ','.join(motions) + '))"'))],
                       node.end_state))

        elif isinstance(node, GuardedChoice):
            # TODO if set(node.guarded_states.expression.free_symbols) <= getProcess().variables
            tree.statements.append(node)
            # else
            # tree.statements.append(ExternalChoice(node.start_state, [x.id for x in node.end_state])

        else:
            tree.statements.append(node)

    return tree


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


class Indirection(DistributedStateNode):

    def __init__(self, start_state, end_state):
        DistributedStateNode.__init__(self, Type.indirection, start_state, end_state)

    def __str__(self):
        return self.start_state[0] + ' = ' + self.end_state[0]

    def accept(self, visitor):
        visitor.visit(self)


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
