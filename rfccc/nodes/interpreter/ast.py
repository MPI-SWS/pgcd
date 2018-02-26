from enum import Enum


class Type(Enum):
    statement = 1
    skip = 2
    send = 3
    receive = 4
    action = 5
    _if = 6
    _while = 7
    assign = 8
    _tuple = 9
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
    _print = 34


class Node:
    def __init__(self, tip):
        self.tip = tip

    def __str__(self):
        return str(self.tip.value)


class Statement(Node):

    def __init__(self, children):
        Node.__init__(self, Type.statement)
        self.children = children

    def __str__(self):
        string = ''
        for c in self.children:
            string += str(c) + '\n'
        return string

    def accept(self, visitor):
        visitor.visit(self)


class Skip(Node):

    def __init__(self):
        Node.__init__(self, Type.skip)

    def __str__(self):
        return 'Skip'

    def accept(self, visitor):
        visitor.visit(self)


class Send(Node):

    def __init__(self, comp, msg_type, value):
        Node.__init__(self, Type.send)
        self.comp = comp
        self.msg_type = msg_type
        self.value = value

    def __str__(self):
        return 'Send(' + str(self.comp) + ',' + str(self.msg_type) + ',' + str(self.value) + ')'

    def accept(self, visitor):
        visitor.visit(self)


class Receive(Node):

    def __init__(self, motion, actions=None):
        Node.__init__(self, Type.receive)
        self.motion = motion
        self.actions = actions

    def __str__(self):
        return 'Receive(' + str(self.motion) + ') {' + ''.join(str(e) for e in self.actions) + '}'

    def accept(self, visitor):
        visitor.visit(self)


class Action(Node):

    def __init__(self, str_msg_type, data_name, program):
        Node.__init__(self, Type.action)
        self.str_msg_type = str_msg_type
        self.data_name = data_name
        self.program = program

    def __str__(self):
        return '(' + str(self.str_msg_type) + ',' + str(self.data_name) + ',' + str(self.program) + ')'

    def accept(self, visitor):
        return visitor.visit(self)


class If(Node):

    def __init__(self, condition, ifCode, elseCode):
        Node.__init__(self, Type._if)
        self.condition = condition
        self.ifCode = ifCode
        self.elseCode = elseCode

    def __str__(self):
        return 'if ' + str(self.condition) + ' then {' + str(self.ifCode) + '} else {' + str(self.elseCode) + '}'

    def accept(self, visitor):
        visitor.visit(self)


class While(Node):

    def __init__(self, condition, code):
        Node.__init__(self, Type._while)
        self.condition = condition
        self.code = code

    def __str__(self):
        return 'while ' + str(self.condition) + ' do {' + str(self.code) + '}'

    def accept(self, visitor):
        visitor.visit(self)


class Assign(Node):

    def __init__(self, id, value, property=None):
        Node.__init__(self, Type.assign)
        self.id = id
        self.value = value
        self.property = property

    def __str__(self):
        if self.property is None:
            return str(self.id) + ' := ' + str(self.value)
        else:
            return str(self.id) + '.' + str(self.property) + ' := ' + str(self.value)

    def accept(self, visitor):
        visitor.visit(self)


class Dict(Node):

    def __init__(self, tup):
        Node.__init__(self, Type._tuple)
        self.tup = tup

    def merge(self, t):
        self.tup = self.merge_two_dicts(self.tup, t.tup)

    def merge_two_dicts(self, x, y):
        """Given two dicts, merge them into a new dict as a shallow copy."""
        z = x.copy()
        z.update(y)
        return z

    def __str__(self):
        return str(self.tup)

    def accept(self, visitor):
        return visitor.visit(self)


class BinOp(Node):

    def __init__(self, operation, sign, exp1, exp2):
        Node.__init__(self, operation)
        self.exp1 = exp1
        self.exp2 = exp2
        self.sign = sign

    def __str__(self):
        return 'expr'

    def __repr__(self):
        return 'expr'

    def accept(self, visitor):
        return visitor.visit(self)


class UnOp(Node):

    def __init__(self, operation, sign, exp):
        Node.__init__(self, operation)
        self.exp = exp
        self.sign = sign

    def __str__(self):
        return 'expr'

    def __repr__(self):
        return 'expr'

    def accept(self, visitor):
        return visitor.visit(self)


class Constant(Node):

    def __init__(self, value):
        Node.__init__(self, Type.expression)
        self.value = value

    def __str__(self):
        return 'expr'

    def __repr__(self):
        return 'expr'

    def accept(self, visitor):
        return visitor.visit(self)


class Motion(Node):

    def __init__(self, value, exps=None):
        Node.__init__(self, Type.motion)
        if exps is None:
            exps = []
        self.value = value
        self.exps = exps

    def __str__(self):
        return ''.join(str(m) for m in self.value)

    def accept(self, visitor):
        visitor.visit(self)
