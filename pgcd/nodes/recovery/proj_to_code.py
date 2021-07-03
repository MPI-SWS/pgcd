import interpreter.ast_inter as ast_inter
from verification.choreography.projection import Projection
from recovery.motion_annot import *
import sympy as sp
import logging
from copy import deepcopy

log = logging.getLogger("Projection2Code")

# turn a recovery choreography into a program
# be carefull about motions. check if it have annotation. if yes, expand
class Proj2Code:

    def __init__(self, choreography, process):
        self.choreography = choreography
        self.process = process
        self.withNextReceive = None

    def getCode(self):
        proj = Projection()
        proj.choreography = deepcopy(self.choreography)
        p_chor = proj.project(self.process.name(), self.process)
        #print(p_chor)
        transformed = []
        state = p_chor.start_state
        state_to_node = p_chor.mk_state_to_node()
        while True:
            node = state_to_node[state]
            if node.isMotion():
                m = node.motions[0]
                assert m.id == self.process.name()
                (name, needIdle, needWait) = decomposeExtraOnly(m)
                if needWait > 0:
                    mi = ast_inter.Motion(name, m.mp_args[:-1])
                    transformed.append(mi)
                    w = ast_inter.Motion('wait', [m.mp_args[-1]])
                    transformed.append(w)
                elif needIdle:
                    mi = ast_inter.Motion(name, m.mp_args)
                    transformed.append(mi)
                    assert self.withNextReceive == None
                else:
                    mi = ast_inter.Motion(name, m.mp_args)
                    # if interruptible should be folded into the next receive !!
                    motion = self.choreography.getMotion(self.process.name(), name, m.mp_args)
                    d = motion.duration()
                    if d.interruptible:
                        assert self.withNextReceive == None
                        self.withNextReceive = mi
                    else:
                        transformed.append(mi)
            elif node.isSendMessage() or (node.isMessage() and node.sender == self.process.name()):
                snd = ast_inter.Send(node.receiver, node.msg_type, node.expressions)
                transformed.append(snd)
            elif node.isReceiveMessage() or (node.isMessage() and node.receiver == self.process.name()):
                m = self.withNextReceive if self.withNextReceive != None else ast_inter.Motion('idle',[])
                self.withNextReceive = None
                rcv = ast_inter.Receive(node.sender, m, [ast_inter.Action(node.msg_type, node.expressions, ast_inter.Skip())])
                transformed.append(rcv)
            elif node.isEnd():
                if self.withNextReceive != None:
                    log.warning(" found end while needing to do: " + str(self.withNextReceive))
                transformed.append(ast_inter.Exit(sp.S(0)))
                return ast_inter.Statement(transformed)
            else:
                assert False, "did not expect " + str(node)
            state = node.end_state[0]
