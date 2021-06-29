import inspect
import threading
import math
import itertools
import logging
from copy import copy
from sympy import S

from verification.choreography.parser_chor import *
from verification.choreography.ast_chor import *
from verification.choreography.ast_proj import *
from verification.choreography.minimize import *
from verification.choreography.normalization import *
from verification.spec.component import Component
from verification.spec.env import Env

log = logging.getLogger("Projection")

class Projection:

    def __init__(self):
        self.choreography = None
        self.artificial_nodes_counter = -1
        self.depth = 0

    def get_artificial_name(self):
        self.artificial_nodes_counter += 1
        return '__x__' + str(self.artificial_nodes_counter)

    def parse(self, code, env = None):
        if isinstance(env, Component):
            env = Env(env)
        parser = ChoreographyParser(env)
        self.choreography = parser.parse(code)
        return self.choreography

    def project(self, proj_name, process):
        log.debug("== Global ==\n%s", self.choreography)
        projection = CreateProjectionFromChoreography(self.choreography, proj_name, process)
        state_to_node = projection.mk_state_to_node()
        log.debug("-- Raw projection (before normalization) --\n%s", projection)
        try:
            self.normalize_projection(projection, state_to_node)
        except Exception as ex:
            logger.error("ERROR during projection, nodes are")
            for v in state_to_node.values():
                logger.error("%s", v)
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
                            if isinstance(node1, ReceiveMessage) and isinstance(node2, ReceiveMessage) and \
                               node1.sender == node2.sender and node1.msg_type == node2.msg_type:
                                raise Exception("ambiguous external choice for message: " + str(node1) + ", " + str(node2))


    def normalize_projection(self, choreography, state_to_node):
        removeIndirections(choreography, state_to_node)
        log.debug("-- after remove indirections --\n%s", choreography)
        removeForkJoin(self, choreography, state_to_node)
        log.debug("-- after remove fork/join --\n%s", choreography)
        minimize(choreography, state_to_node)
        log.debug("-- after minimize --\n%s", choreography)
        self.checkProjection(choreography, state_to_node)
