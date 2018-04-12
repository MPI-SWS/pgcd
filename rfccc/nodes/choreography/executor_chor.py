import inspect
import threading
import math
import itertools
from copy import copy
from sympy import S

from parser_chor import *
from ast_chor import *
from ast_proj import *
from normalization import *


class ChoreographyExecutor:

    def __init__(self):
        self.parser = ChoreographyParser()
        self.choreography = None
        self.artificial_nodes_counter = -1
        self.depth = 0

    def get_artificial_name(self):
        self.artificial_nodes_counter += 1
        return '__x__' + str(self.artificial_nodes_counter)

    def execute(self, code, world = None):
        self.choreography = self.parser.parse(code, world)
        return self.choreography

    def project(self, proj_name, process, debug = False):
        if debug:
            print("== Global ==")
            print(self.choreography)
        projection = CreateProjectionFromChoreography(self.choreography, proj_name, process)
        state_to_node = projection.mk_state_to_node()
        if debug:
            print("-- Raw projection (before normalization) --")
            print(projection)
        try:
            self.normalize_projection(projection, state_to_node, debug)
        except Exception as ex:
            print("ERROR during projection, nodes are")
            for v in state_to_node.values():
                print(v)
            raise
        return projection

    def checkProjection(self, choreography, state_to_node):
        def nextActionIsExternal(node):
            for n1 in node.end_state:
                node1 = state_to_node[n1]
                if isinstance(node1, ReceiveMessage) or isinstance(node1, Motion):
                    return True
                elif isinstance(node1, SendMessage) or isinstance(node1, GuardedChoice):
                    return False
                else:
                    return all(nextActionIsExternal(state_to_node[s]) for s in node1.end_state)
        for node in state_to_node.values():
            if isinstance(node, ExternalChoice):
                assert nextActionIsExternal(node), "external choice followed by internal action: " + str(node)
                for n1 in node.end_state:
                    node1 = state_to_node[n1]
                    for n2 in node.end_state:
                        if n1 < n2:
                            node2 = state_to_node[n2]
                            if isinstance(node1, Motion) and isinstance(node2, Motion):
                                raise Exception("ambiguous external choice for motion: " + str(node1) + ", " + str(node2))
                            if isinstance(node1, ReceiveMessage) and isinstance(node2, ReceiveMessage) and node1.msg_type == node2.msg_type:
                                raise Exception("ambiguous external choice for message: " + str(node1) + ", " + str(node2))


    def normalize_projection(self, choreography, state_to_node, debug = False):
        removeIndirections(choreography, state_to_node)
        if debug:
            print("-- after remove indirections --")
            print(choreography)
        removeForkJoin(self, choreography, state_to_node, debug)
        if debug:
            print("-- after remove fork/join --")
            print(choreography)
        #to help the refinement
        #addExternalChoice(self, choreography, state_to_node, debug)
        #if debug:
        #    print("-- after add external --")
        #    print(choreography)
        minimize(choreography, state_to_node, debug)
        if debug:
            print("-- after minimize --")
            print(choreography)
        self.checkProjection(choreography, state_to_node)
