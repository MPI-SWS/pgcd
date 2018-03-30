import inspect
import threading
import math

from parser_chor import *
from ast_chor import *
from ast_proj import *


class ChoreographyExecutor:

    def __init__(self):
        self.parser = ChoreographyParser()

    def execute(self, code):
        tree = self.parser.parse(code)
        CreateProjectionFromChoreography(tree, "cart", "cart")

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
            pass


    def visit_choreography(self, node):
        pass

    def visit_message(self, node):
        pass

    def visit_motion(self, node):
        pass

    def visit_motion_arg(self, node):
        pass

    def visit_guard(self, node):
        pass

    def visit_guard_arg(self, node):
        pass

    def visit_merge(self, node):
        pass

    def visit_fork(self, node):
        pass

    def visit_join(self, node):
        pass

