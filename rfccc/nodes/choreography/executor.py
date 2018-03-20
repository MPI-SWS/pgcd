import inspect
import threading
import math

from choreography.parser import *
from choreography.ast import *


class ChoreographyExecutor:

    def __init__(self):
        self.parser = ChoreographyParser()

    def execute(self, code):
        self.parser.parse(code).accept(self)

    def visit(self, node):

        if isinstance(node, Choreography):
            self.visit_choreography(node)

        elif isinstance(node, Statements):
            self.visit_statement(node)

        elif isinstance(node, Message):
            self.visit_message(node)

        elif isinstance(node, Motion):
            self.visit_motion(node)

        elif isinstance(node, MotionArg):
            self.visit_motion_arg(node)

        elif isinstance(node, Guard):
            self.visit_guard(node)

        elif isinstance(node, GuardArg):
            self.visit_guard_arg(node)

        elif isinstance(node, Merge):
            self.visit_merge(node)

        elif isinstance(node, Fork):
            self.visit_fork(node)

        elif isinstance(node, Join):
            self.visit_join(node)

        elif isinstance(node, End):
            print(node.start_state + '=' + 'end')

        elif isinstance(node, BinOp):
            print("expression", end='')
            #print(self.visit_bin_op(node), end='')

        elif isinstance(node, UnOp):
            print("expression", end='')
            #print(self.visit_un_op(node), end='')

        elif isinstance(node, Constant):
            print("expression", end='')
            #print(node.value, end='')

    def visit_choreography(self, node):
        print("def ", end='')
        node.statements.accept(self)
        print(" in " + "[" , end='')
        node.predicate.accept(self)
        print( "]" + str(node.start_state))

    def visit_statement(self, node):
        for stmt in node.children:
            stmt.accept(self)
            print("\n")

    def visit_message(self, node):
        print(node.start_state + '=' + node.comp1 + '->'
              + node.comp2 + ':' + node.msg_type + '(', end='')
        for x in node.expressions:
            x.accept(self)
            if x != node.expressions[-1]:
                print(',', end='')
        print( ');' + node.end_state, end='')

    def visit_motion(self, node):
        print(node.start_state + '= (', end='')
        for x in node.motions:
            x.accept(self)
            if x != node.motions[-1]:
                print(',', end='')
        print('); ' + node.end_state, end='')

    def visit_motion_arg(self, node):
        print(node.id + ': ', end='')
        print(node.motion_name + '(', end='')
        for x in node.motion_params:
            x.accept(self)
            if x != node.motion_params[-1]:
                print(',', end='')
        print(')', end='')

    def visit_guard(self, node):
        print(node.start_state + '=', end='')
        for x in node.guarded_states:
            x.accept(self)
            if x != node.guarded_states[-1]:
                print('+', end='')

    def visit_guard_arg(self, node):
        print('[' , end='')
        node.expression.accept(self)
        print( ']' + node.id, end='')

    def visit_merge(self, node):
        for x in node.merge_states:
            print(x, end='')
            if x != node.merge_states[-1]:
                print('+', end='')
        print('=' + node.end_state, end='')

    def visit_fork(self, node):
        print(node.start_state + '=', end='')
        for x in node.forked_states:
            print(x, end='')
            if x != node.forked_states[-1]:
                print('||', end='')

    def visit_join(self, node):
        for x in node.joined_states:
            print(x, end='')
            if x != node.joined_states[-1]:
                print('+', end='')
        print('=' + node.end_state, end='')

    def visit_bin_op(self, node):
        var1 = node.exp1.accept(self)
        var2 = node.exp2.accept(self)
        if isinstance(var1, str) or isinstance(var2, str):
            var1 = '"' + str(var1) + '"'
            var2 = '"' + str(var2) + '"'
        return eval(str(var1) + ' ' + str(node.sign) + ' ' + str(var2))

    def visit_un_op(self, node):
        var1 = node.exp.accept(self)
        if isinstance(var1, str):
            var1 = '"' + str(var1) + '"'
        return eval(str(node.sign) + ' (' + str(var1) + ') ')
