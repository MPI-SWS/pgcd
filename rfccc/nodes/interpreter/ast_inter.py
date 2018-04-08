from enum import Enum
from sympy import *

class Type(Enum):
    statement = 1
    skip = 2
    send = 3
    receive = 4
    action = 5
    _if = 6
    _while = 7
    assign = 8
    expression = 10
    motion = 33
    _print = 34



class Node:

    label_num = -1

    def __init__(self, tip):
        self.tip = tip

    def __str__(self):
        return str(self.tip.value)

    def get_label():
        Node.label_num += 1
        return 'X' + Node.label_num

    def label_as_root(self):
        label_to_node = {}
        self.label(label_to_node)
        return label_to_node

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

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()
        for c in self.children:
            c.label(label_to_node)

class Skip(Node):

    def __init__(self):
        Node.__init__(self, Type.skip)

    def __str__(self):
        return 'Skip'

    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()

class Send(Node):

    def __init__(self, comp, msg_type, args):
        Node.__init__(self, Type.send)
        self.comp = comp
        self.msg_type = msg_type
        self.args = args

    def __str__(self):
        return 'Send(' + str(self.comp) + ',' + str(self.msg_type) + ',' + ''.join(self.args) + ')'

    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()


class Receive(Node):

    def __init__(self, motion, actions=None):
        Node.__init__(self, Type.receive)
        self.motion = motion
        self.actions = actions

    def __str__(self):
        return 'Receive(' + str(self.motion) + ') {' + ''.join(str(e) for e in self.actions) + '}'

    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()
        self.motion.label(label_to_node)
        for c in self.actions:
            c.label(label_to_node)


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

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()
        program.label(label_to_node)

class If(Node):

    def __init__(self, condition, ifCode, elseCode):
        Node.__init__(self, Type._if)
        self.if_list = []
        self.if_list += self.flatten(condition, ifCode)
        self.if_list += self.flatten(Not(condition), elseCode)

    def __str__(self):
        return ''.join(self.if_list)

    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        for if_stmt in self.if_list:
            if_stmt.label(label_to_node)

    def flatten(self, condition, program):
        ft_child = program.children[0]
        if not isinstance(ft_child, If):
            return [IfComponent(condition, program)]
        else:
            program.children.pop(0)
            ifs = []
            for if_stmt in ft_child.if_list:
                cond = And(condition, if_stmt.condition)
                prog = if_stmt.program + program
                ifs.append(IfComponent(cond, prog))
            return ifs


class IfComponent(Node):

    def __init__(self, condition, program):
        Node.__init__(self, Type._if)
        self.condition = condition
        self.program = program

    def __str__(self):
        return 'if ' + str(self.condition) + ' then {' + str(self.program) + '}'

    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()
        program.label(label_to_node)

class While(Node):

    def __init__(self, condition, program):
        Node.__init__(self, Type._while)
        self.condition = condition
        self.program = program

    def __str__(self):
        return 'while ' + str(self.condition) + ' do {' + str(self.program) + '}'

    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()
        program.label(label_to_node)

class Assign(Node):

    def __init__(self, id, value):
        Node.__init__(self, Type.assign)
        self.id = id
        self.value = value

    def __str__(self):
        return str(self.id) + ' := ' + str(self.value)


    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        pass


class Motion(Node):

    def __init__(self, value, exps=[]):
        Node.__init__(self, Type.motion)
        self.value = value
        self.exps = exps

    def __str__(self):
        return ''.join(str(m) for m in self.value)

    def accept(self, visitor):
        visitor.visit(self)

    def label(self, label_to_node):
        label_to_node[self] = self.get_label()