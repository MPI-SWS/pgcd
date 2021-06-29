from enum import Enum
from sympy import *

class Type(Enum):
    statement = 1 # TODO rename as block
    skip = 2
    send = 3
    receive = 4
    action = 5
    _if = 6
    _while = 7
    assign = 8
    expression = 9
    motion = 10
    _print = 11
    exit = 12
    checkpoint = 13



class AstNode:

    label_num = -1

    def __init__(self, tip):
        self.tip = tip
        self._label = None

    def __str__(self):
        return str(self.tip.value)

    def set_label(self, label):
        assert (self._label == None)
        self._label = label

    def get_label(self):
        if (self._label == None):
            AstNode.label_num += 1
            self._label = 'L' + str(AstNode.label_num)
        return self._label

    def label_as_root(self):
        label_to_node = {}
        self.label(label_to_node)
        return label_to_node

    def label(self, label_to_node):
        label_to_node[self.get_label()] = self

    def isBlock(self):
        return self.tip == Type.statement

    def isMotion(self):
        return self.tip == Type.motion

    def isChoice(self):
        return self.tip == Type._if or self.tip == Type._while

    def isIf(self):
        return self.tip == Type._if

    def isWhile(self):
        return self.tip == Type._while

    def isSend(self):
        return self.tip == Type.send

    def isReceive(self):
        return self.tip == Type.receive

    def isExit(self):
        return self.tip == Type.exit

    def isCheckpoint(self):
        return self.tip == Type.checkpoint

    def contains(self, node):
        return self == node

    def exists(self, f):
        return f(self)

class Statement(AstNode):

    def __init__(self, children):
        AstNode.__init__(self, Type.statement)
        self.children = children

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        for c in self.children:
            string += str(c) + '\n'
        return string

    def label(self, label_to_node):
        super().label(label_to_node)
        for c in self.children:
            c.label(label_to_node)

    def contains(self, node):
        return self == node or any(c.contains(node) for c in self.children)

    def exists(self, f):
        return f(self) or any(c.exists(f) for c in self.children)

class Skip(AstNode):

    def __init__(self):
        AstNode.__init__(self, Type.skip)

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'Skip'
        return string

class Print(AstNode):

    def __init__(self, arg):
        AstNode.__init__(self, Type._print)
        self.arg = arg

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'Print' + str(arg)
        return string

class Send(AstNode):

    def __init__(self, comp, msg_type, args):
        AstNode.__init__(self, Type.send)
        self.comp = comp
        self.msg_type = msg_type
        self.args = args

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'Send(' + str(self.comp) + ',' + str(self.msg_type) + ',' + ''.join([str(a) for a in self.args]) + ')'
        return string


class Receive(AstNode):

    def __init__(self, sender, motion, actions=None):
        AstNode.__init__(self, Type.receive)
        self.sender = sender
        self.motion = motion
        self.actions = actions

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'Receive(' + self.sender + ', ' + str(self.motion) + ') {' + ''.join(str(e) for e in self.actions) + '}'
        return string

    def label(self, label_to_node):
        super().label(label_to_node)
        self.motion.label(label_to_node)
        for c in self.actions:
            c.label(label_to_node)

    def contains(self, node):
        return self == node or self.motion.contains(node) or any(c.contains(node) for c in self.actions)

    def exists(self, f):
        return f(self) or self.motion.exists(f) or any(c.exists(f) for c in self.children)

class Action(AstNode):

    def __init__(self, str_msg_type, data_names, program):
        AstNode.__init__(self, Type.action)
        self.str_msg_type = str_msg_type
        self.data_names = data_names
        self.program = program

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'case ' + str(self.str_msg_type) + '('
        string += ','.join([ str(d) for d in self.data_names])
        string += ') =>' + str(self.program)
        return string

    def label(self, label_to_node):
        super().label(label_to_node)
        self.program.label(label_to_node)

    def contains(self, node):
        return self == node or self.program.contains(node)

    def exists(self, f):
        return f(self) or self.program.exists(f)

class If(AstNode):

    def __init__(self, condition, ifCode, elseCode):
        AstNode.__init__(self, Type._if)
        self.if_list = []
        self.if_list += self.flatten(condition, ifCode)
        self.if_list += self.flatten(Not(condition), elseCode)

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += '\n'.join([str(a) for a in self.if_list])
        return string

    def label(self, label_to_node):
        super().label(label_to_node)
        for if_stmt in self.if_list:
            if_stmt.label(label_to_node)

    def flatten(self, condition, statement):
        if not isinstance(statement, If):
            return [IfComponent(condition, statement)]
        else:
            ifs = []
            for if_stmt in statement.if_list:
                cond = And(condition, if_stmt.condition)
                prog = Statement(if_stmt.program.children + statement.children)
                ifs.append(IfComponent(cond, prog))
            return ifs

    def contains(self, node):
        return self == node or any(i.contains(node) for i in self.if_list)

    def exists(self, f):
        return f(self) or any(c.exists(f) for c in self.if_list)


class IfComponent(AstNode):

    def __init__(self, condition, program):
        AstNode.__init__(self, Type._if)
        self.condition = condition
        self.program = program

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'if ' + str(self.condition) + ' then {' + str(self.program) + '}'
        return string

    def label(self, label_to_node):
        super().label(label_to_node)
        self.program.label(label_to_node)

    def contains(self, node):
        return self == node or self.program.contains(node)

    def exists(self, f):
        return f(self) or self.program.exists(f)

class While(AstNode):

    def __init__(self, condition, program):
        AstNode.__init__(self, Type._while)
        self.condition = condition
        self.program = program

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'while ' + str(self.condition) + ' do {' + str(self.program) + '}'
        return string

    def label(self, label_to_node):
        super().label(label_to_node)
        self.program.label(label_to_node)

    def contains(self, node):
        return self == node or self.program.contains(node)

    def exists(self, f):
        return f(self) or self.program.exists(f)

class Assign(AstNode):

    def __init__(self, id, value):
        AstNode.__init__(self, Type.assign)
        self.id = id
        self.value = value

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += str(self.id) + ' := ' + str(self.value)
        return string


class Motion(AstNode):

    def __init__(self, value, exps=[]):
        AstNode.__init__(self, Type.motion)
        self.value = value
        self.exps = exps

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'm_' + self.value + '(' + ', '.join(str(e) for e in self.exps) + ')'
        return string

class Exit(AstNode):

    def __init__(self, expr):
        AstNode.__init__(self, Type.exit)
        self.expr = expr

    def __str__(self):
        string = ''
        if self._label != None:
            string += self._label + ": "
        string += 'exit(' + str(self.expr) + ')'
        return string

class Checkpoint(AstNode):

    def __init__(self, ids):
        AstNode.__init__(self, Type.checkpoint)
        self.ids = ids

    def __str__(self):
        return 'checkpoint'
