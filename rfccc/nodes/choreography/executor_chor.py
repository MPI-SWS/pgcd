import inspect
import threading
import math

from parser_chor import *
from ast_chor import *


class ChoreographyExecutor:

    def __init__(self):
        self.parser = ChoreographyParser()

    def execute(self, code):
        self.parser.parse(code).accept(self)

    def visit(self, node):

        if isinstance(node, Choreography):
            self.visit_choreography(node)

        elif isinstance(node, Message):
            self.visit_message(node)

        elif isinstance(node, Motion):
            self.visit_motion(node)

        elif isinstance(node, MotionArg):
            self.visit_motion_arg(node)

        elif isinstance(node, GuardedChoice):
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
            print(''.join(node.start_state) + '=' + 'end')


    def visit_choreography(self, node):
        print("def ", end='')
        for stmt in node.statements:
            stmt.accept(self)
            print("\n")
        print(" in " + "[" , end='')
        print(str(node.predicate), end='')
        print( "]" + str(node.start_state))

    def visit_message(self, node):
        print(''.join(node.start_state) + '=' + node.comp1 + '->'
              + node.comp2 + ':' + node.msg_type + '(', end='')
        for x in node.expressions:
            print(x, end='')
            if x != node.expressions[-1]:
                print(',', end='')
        print( ');' + ''.join(node.end_state), end='')

    def visit_motion(self, node):
        print(''.join(node.start_state) + '= (', end='')
        for x in node.motions:
            x.accept(self)
            if x != node.motions[-1]:
                print(',', end='')
        print('); ' + ''.join(node.end_state), end='')

    def visit_motion_arg(self, node):
        print(node.id + ': ', end='')
        print(node.sympy_formula, end='')
        print(')', end='')

    def visit_guard(self, node):
        print(''.join(node.start_state) + '=', end='')
        for x in node.guarded_states:
            x.accept(self)
            if x != node.guarded_states[-1]:
                print('+', end='')

    def visit_guard_arg(self, node):
        print('[' , end='')
        print(node.expression, end='')
        print( ']' + node.id, end='')

    def visit_merge(self, node):
        for x in node.start_state:
            print(x, end='')
            if x != node.start_state[-1]:
                print('+', end='')
        print('=' + ''.join(node.end_state), end='')

    def visit_fork(self, node):
        print(''.join(node.start_state) + '=', end='')
        for x in node.end_state:
            print(x, end='')
            if x != node.end_state[-1]:
                print('||', end='')

    def visit_join(self, node):
        for x in node.start_state:
            print(x, end='')
            if x != node.start_state[-1]:
                print('||', end='')
        print('=' + ''.join(node.end_state), end='')

