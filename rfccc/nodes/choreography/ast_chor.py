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
    plus = 11
    minus = 12
    times = 13
    divide = 14
    mod = 15
    _and = 16
    _or = 17
    gt = 18
    ge = 19
    lt = 20
    le = 21
    eq = 22
    ne = 23
    uminus = 24
    _sin = 25
    _cos = 26
    _tan = 27
    _abs = 28
    _sqrt = 29
    _not = 30
    id = 31
    dot = 32
    motion = 33
    end = 34


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
        string = 'choreography ' + self.id
        return string

    def accept(self, visitor):
        visitor.visit(self)


class Message(DistributedStateNode):

    def __init__(self, start_state, comp1, comp2, msg_type, expressions, continue_state):
        DistributedStateNode.__init__(self, Type.message, start_state, continue_state)

        if comp1 == comp2:
            raise Exception("No self message! (" + comp1 + "->" + comp2 + ")")

        # TODO : this is only for testing purposes here,
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
        return string

    def accept(self, visitor):
        visitor.visit(self)


class Motion(DistributedStateNode):

    def __init__(self, start_state, motions, end_state):
        DistributedStateNode.__init__(self, Type.motion, start_state, end_state)
        self.motions = motions

    def __str__(self):
        return 'Motion'

    def accept(self, visitor):
        visitor.visit(self)


class MotionArg(Node):

    def __init__(self, id, motion_name, motion_params):
        Node.__init__(self, Type.motion_arg)
        self.id = id
        self.motion_name = motion_name
        self.motion_params = motion_params

        # if not self.is_debug and len(Choreography.initialized_components) != 0:
        #     if motion_name not in Choreography.initialized_components:
        #         raise Exception("Component with name '" + motion_name + "' is not in programs processes.")
        # else:
        Choreography.initialized_components.add(motion_name)

    def __str__(self):
        return 'MotionArg'

    def accept(self, visitor):
        visitor.visit(self)


class Guard(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.guard, start_state, [x.id for x in continue_state])
        self.guarded_states = continue_state

    def __str__(self):
        return 'Guard'

    def accept(self, visitor):
        visitor.visit(self)


class GuardArg(Node):

    def __init__(self, expression, id):
        Node.__init__(self, Type.guard_arg)
        self.id = id
        self.expression = expression

    def __str__(self):
        return 'GuardArg'

    def accept(self, visitor):
        visitor.visit(self)


class Merge(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.merge, start_state, continue_state)

    def __str__(self):
        return 'Merge'

    def accept(self, visitor):
        visitor.visit(self)


class Fork(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.merge, start_state, continue_state)

    def __str__(self):
        return 'Fork'

    def accept(self, visitor):
        visitor.visit(self)


class Join(DistributedStateNode):

    def __init__(self, start_state, continue_state):
        DistributedStateNode.__init__(self, Type.merge, start_state, continue_state)
        self.check_states = []

    def __str__(self):
        return 'Join'

    def accept(self, visitor):
        visitor.visit(self)


class End(DistributedStateNode):

    def __init__(self, start_state):
        DistributedStateNode.__init__(self, Type.end, start_state, [])

    def __str__(self):
        return 'End'

    def accept(self, visitor):
        visitor.visit(self)
