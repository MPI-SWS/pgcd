from ast_chor import *
from enum import Enum
from sympy import *


class Type(Enum):
    projection = 0
    statement = 1
    message = 2
    merge = 3
    join = 4
    fork = 5
    guard_arg = 6
    guard = 7
    join_fork_arg = 8
    motion_arg = 9
    expression = 10
    motion = 33
    end = 34

def CreateProjectionFromChoreography(choreography, _id, process):
    tree = ChoreographyProjection(_id, [], choreography.start_state, choreography.end_state, process)

    for node in choreography.statements:

        if isinstance(node, Message):
            if node.comp1 == process:
                tree.statements.add(SendMessage(node.start_state, node.comp2, node.msg_type, node.expressions, node.end_state))
            elif node.comp2 == process:
                tree.statements.add(ReceiveMessage(node.start_state, node.msg_type, node.expressions, node.end_state))
            else:
                tree.statements.add(Indirection(node.start_state, node.end_state))

        elif isinstance(node, Motion):
            motions = []
            for x in node.motions:
                if x.id == process:
                    tree.statements.add(node)
                    continue
                motions.append(str(x.sympy_formula).split('(')[0])
            tree.statements.add(Motion(node.start_state, [MotionArg(process, sympify('"wait(duration('+ ','.join(motions) + '))"') )], node.end_state))

        # elif isinstance(node, GuardedChoice):
        #
        # elif isinstance(node, Merge):
        #
        # elif isinstance(node, Fork):
        #
        # elif isinstance(node, Join):
        #
        # elif isinstance(node, End):
        #     print(''.join(node.start_state) + '=' + 'end')


class ChoreographyProjection(DistributedStateNode):

    def __init__(self, id, statements, predicate, start_state, process):
        DistributedStateNode.__init__(self, Type.projection, start_state, None)
        self.id = id
        self.statements = statements
        self.predicate = predicate
        self.start_state = start_state
        self.process = process

    def __str__(self):
        string = 'choreography projection' + self.id
        return string

    def accept(self, visitor):
        visitor.visit(self)


class SendMessage(DistributedStateNode):

    def __init__(self, start_state, receiver, msg_type, expressions, continue_state):
        DistributedStateNode.__init__(self, Type.message, start_state, continue_state)
        self.receiver = receiver
        self.msg_type = msg_type
        self.expressions = expressions

    def __str__(self):
        string = 'SendMessage'
        return string

    def accept(self, visitor):
        visitor.visit(self)

class ReceiveMessage(DistributedStateNode):

    def __init__(self, start_state, msg_type, expressions, continue_state):
        DistributedStateNode.__init__(self, Type.message, start_state, continue_state)
        self.msg_type = msg_type
        self.expressions = expressions

    def __str__(self):
        string = 'ReceiveMessage'
        return string

    def accept(self, visitor):
        visitor.visit(self)

class Indirection(DistributedStateNode):

    def __init__(self, start_state, end_state):
        DistributedStateNode.__init__(self, Type.motion, start_state, end_state)

    def __str__(self):
        return 'Indirection'

    def accept(self, visitor):
        visitor.visit(self)

class InternalChoice(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.guard, start_state, [x.id for x in continue_state])
        self.guarded_states = continue_state

    def __str__(self):
        return 'InternalChoice'

    def accept(self, visitor):
        visitor.visit(self)

class ExternalChoice(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.guard, start_state, [x.id for x in continue_state])
        self.guarded_states = continue_state

    def __str__(self):
        return 'ExternalChoice'

    def accept(self, visitor):
        visitor.visit(self)























