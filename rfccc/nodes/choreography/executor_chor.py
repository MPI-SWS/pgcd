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
        self.sequence = self.parser.parse(code)
        return self.sequence

    def project(self, proj_name,  process):
        CreateProjectionFromChoreography(self.sequence, "cart", "cart")