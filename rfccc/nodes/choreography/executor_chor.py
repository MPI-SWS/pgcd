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
            return

        elif isinstance(node, ExternalChoice):
            node_type = state_to_node[node.end_state[0]].tip
            nodes = []
            prev = None
            eq_types = True
            for choice in node.end_state:
                nodes.add(state_to_node[choice])
                if prev is not None or not prev.shift_delay_check(state_to_node[choice]):
                    eq_types = False
                    break
                prev = state_to_node[choice]
            if len(nodes) == 1 and nodes[0].tip == Type.merge and len(
                    set(node.start_state) - set(nodes[0].end_state)) == 0:
                state_to_node[state] = Indirection([state], [nodes[0].end_state])
            elif eq_types:   # x0 = x1 & x2 | x1 = Y; x3 | x2 = Y; x4
                merge_list = [x.end_state[0] for x in nodes] # x3, x4
                modif = nodes[0] # x1 = Y; x3
                modif.start_state = node.start_state[0] # x0 = Y; x3
                modif.end_state = node.end_start[0]  # x0 = Y; x1
                state_to_node[state] = modif # x0 -> x0 = Y; x1
                #--------------------------------------------------------
                ex_choice = node # x0 = x1 & x2
                ex_choice.end_state = merge_list # x0 = x3 & x4
                ex_choice.start_state = modif.end_state # x1 = x3 & x4
                state_to_node[modif.end_state[0]] = ex_choice # x1 -> x1 = x3 & x4
            for st in modif.end_state[0]:
                self.normalize_projection(st, state_to_node)
            return

        elif isinstance(node, Fork):
            for fork in node.end_state:
                self.normalize_projection(fork, state_to_node)





















