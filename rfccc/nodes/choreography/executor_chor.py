import inspect
import threading
import math

from parser_chor import *
from ast_chor import *
from ast_proj import *


class ChoreographyExecutor:

    def __init__(self):
        self.parser = ChoreographyParser()
        self.choreography = None

    def execute(self, code):
        self.choreography = self.parser.parse(code)
        return self.choreography

    def project(self, proj_name, process):
        projection, state_to_node = CreateProjectionFromChoreography(self.choreography, proj_name, process)
        return self.normalize_projection(projection.start_state[0], state_to_node)

    def normalize_projection(self, state, state_to_node):

        node = state_to_node[state]

        if isinstance(node, Indirection):
            state_to_node[node.start_state[0]] = state_to_node[node.end_state[0]]
            self.normalize_projection(node.end_state[0], state_to_node)

        elif isinstance(node, ExternalChoice):
            nodes = set()
            for choice in node.end_state:
                nodes.add(state_to_node[choice])
            if len(nodes) == 1:
                for choice in node.end_state:
                    del state_to_node[choice]
                state_to_node[state] = nodes.pop()
                state_to_node[state_to_node[state].end_state[0]] = node


            self.normalize_projection(node.end_state[0], state_to_node)


