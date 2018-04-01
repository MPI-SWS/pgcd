from email.policy import strict
from enum import Enum
from interpreter.ast_inter import Node


class Type(Enum):
    choreography = 0
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
    projection = 40
    send_message = 41
    receive_message = 42
    indirection = 43
    external_choice = 44


class DistributedStateNode(Node):

    def __init__(self, tip, start_state, end_state):
        Node.__init__(self, tip)
        self.start_states_reached = set()
        self.end_states_reached = set()
        self.start_state = start_state
        self.end_state = end_state

    def __str__(self):
        return Node.__str__(self) + ": " + str(self.start_state) + " -> " + str(self.end_state)


class Choreography(DistributedStateNode):
    initialized_components = set()

    def __init__(self, id, statements, predicate, start_state):
        DistributedStateNode.__init__(self, Type.choreography, start_state, None)
        self.id = id
        self.statements = statements
        self.predicate = predicate
        self.start_state = start_state

    def __str__(self):
        string = "def "+ self.id +" \n"
        for stmt in self.statements:
            string += str(stmt) + '\n'
        string += " in [" + str(self.predicate) + ']' + str(self.start_state)
        return string

    def accept(self, visitor):
        visitor.visit(self)


class Message(DistributedStateNode):

    def __init__(self, start_state, comp1, comp2, msg_type, expressions, continue_state):
        DistributedStateNode.__init__(self, Type.message, start_state, continue_state)

        if comp1 == comp2:
            raise Exception("No self message! (" + comp1 + "->" + comp2 + ")")

        # TODO : uncomment in production mode
        # when the main.py launches self.initialized_components
        # will be populated!
        # if not self.is_debug and len(Choreography.initialized_components) != 0:
        #     if comp1 not in Choreography.initialized_components:
        #         raise Exception("Component with name '" + comp1 + "' is not in programs processes.")
        #     if comp2 not in Choreography.initialized_components:
        #         raise Exception("Component with name '" + comp2 + "' is not in programs processes.")
        # else:
        Choreography.initialized_components.add(comp1)
        Choreography.initialized_components.add(comp2)

        self.comp1 = comp1
        self.comp2 = comp2
        self.msg_type = msg_type
        self.expressions = expressions

    def __str__(self):
        string = 'Message'
        string += self.start_state[0] + '=' + self.comp1 + '->' + self.comp2 + ':' + self.msg_type + '('
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
        string = ''.join(self.start_state) + '= ('
        for x in self.motions:
            string += str(x)
            if x != self.motions[-1]:
                string += ','
        string += '); ' + ''.join(self.end_state)
        return string

    def accept(self, visitor):
        visitor.visit(self)


class MotionArg(Node):

    def __init__(self, id, sympy_formula):
        Node.__init__(self, Type.motion_arg)
        self.id = id
        self.sympy_formula = sympy_formula

        # if not self.is_debug and len(Choreography.initialized_components) != 0:
        #     if motion_name not in Choreography.initialized_components:
        #         raise Exception("Component with name '" + motion_name + "' is not in programs processes.")
        # else:
        Choreography.initialized_components.add(id)

    def __str__(self):
        string = self.id + ': '
        string += self.sympy_formula + ')'
        return string

    def accept(self, visitor):
        visitor.visit(self)


class GuardedChoice(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.guard, start_state, [x.id for x in continue_state])
        self.guarded_states = continue_state

    def __str__(self):
        string = ''.join(self.start_state) + '='
        for x in self.guarded_states:
            string += str(x)
            if x != self.guarded_states[-1]:
                string += '+'
        return string

    def accept(self, visitor):
        visitor.visit(self)


class GuardArg(Node):

    def __init__(self, expression, id):
        Node.__init__(self, Type.guard_arg)
        self.id = id
        self.expression = expression

    def __str__(self):
        string = '['
        string += str(self.expression)
        string += ']' + str(self.id)
        return string

    def accept(self, visitor):
        visitor.visit(self)


class Merge(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.merge, start_state, continue_state)

    def __str__(self):
        string = ''
        for x in self.start_state:
            string += str(x)
            if x != self.start_state[-1]:
                string += ' + '
        string += '=' + ''.join(self.end_state)

    def accept(self, visitor):
        visitor.visit(self)


class Fork(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.fork, start_state, continue_state)

    def __str__(self):
        string = ''.join(self.start_state) + '='
        for x in self.end_state:
            string += str(x)
            if x != self.end_state[-1]:
                string += '||'
        return string

    def accept(self, visitor):
        visitor.visit(self)


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
        string += '=' + ''.join(self.end_state)
        return string

    def accept(self, visitor):
        visitor.visit(self)


class End(DistributedStateNode):

    def __init__(self, start_state):
        DistributedStateNode.__init__(self, Type.end, start_state, [])

    def __str__(self):
        return ''.join(self.start_state) + '=' + 'end'

    def accept(self, visitor):
        visitor.visit(self)
