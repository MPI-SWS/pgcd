from parser_chor import *
from ast_chor import *
from ast_proj import *
import ast_inter
from sympy import *
from DrealInterface import DrealInterface
from vc import VC
from copy import *

#the refinement check between a program and the projection

class Refinement():

    def __init__(self, program, projection, debug = False):
        self.debug = debug
        self.program = program
        self.projection = projection
        self.state_to_node = projection.mk_state_to_node()
        self.cachedImplication = {}
        self.programLabels = program.label_as_root()
        self.initLabel = program.get_label()
        if debug:
            print("= Program =")
            print(program)
            print("> starting with", self.initLabel)
        self.nextLabel = { l:set() for l in self.programLabels }
        self.buildCFA([], program)
        if debug:
            print("= CFA =")
            for l, s in self.nextLabel.items():
                print(l, "->", s)
        self.compat = {}

    def buildCFA(self, lastLabels, statment):
        #connect prev
        l = statment.get_label()
        for ls in lastLabels:
            self.nextLabel[ls].add(l)
        lastLabel = [l]
        # dig deeper
        if isinstance(statment, ast_inter.Statement):
            for i in range(0, len(statment.children)):
                lastLabel = self.buildCFA(lastLabel, statment.children[i])
        elif isinstance(statment, ast_inter.Receive):
            successors = [statment.motion] + statment.actions
            lastLabel = { l for i in successors for l in self.buildCFA(lastLabel, i) }
        elif isinstance(statment, ast_inter.Action):
            lastLabel = self.buildCFA(lastLabel, statment.program)
        elif isinstance(statment, ast_inter.If):
            lastLabel = { l for i in statment.if_list for l in self.buildCFA(lastLabel, i) }
        elif isinstance(statment, ast_inter.IfComponent):
            lastLabel = self.buildCFA(lastLabel, statment.program)
        elif isinstance(statment, ast_inter.While):
            ls2 = self.buildCFA([], statment.program) # trick: the next of a while is the else case
            for l2 in ls2:
                self.nextLabel[l2].add(l)
        return lastLabel

    def implies(self, cond1, cond2):
        if (cond1,cond2) in self.cachedImplication:
            return self.cachedImplication[(cond1,cond2)]
        else:
            vc = VC("implication", And(cond1, Not(cond2)))
            res = vc.discharge()
            self.cachedImplication[(cond1,cond2)] = res
            return res

    def _refines(self, statment, node):
        return node in self.compat[statment]

    def nextStatement(self, statment):
        n = self.nextLabel(statment)
        if len(n) == 0:
            raise Exception("no successors for " + str(statment))
        elif len(n) > 1:
            raise Exception("ambiguous successors: " + n + " for " + str(statment))
        else:
            for elt in n:
                return elt

    def sameMpName(self, n1, n2):
        return n1 == n2 or n1 == ('m_' + n2)
    
    def sameMsgName(self, n1, n2):
        return n1 == n2 or n1 == ('msg_' + n2)

    def compatible(self, statmentL, nodeL):
        statment = self.programLabels[statmentL]
        node = self.state_to_node[nodeL]
        if self.debug:
            print(">", statment)
            print(">", node)
        if isinstance(statment, ast_inter.Motion):
            return isinstance(node, Motion) and self.sameMpName(statment.value, node.motions[0].mp_name) #TODO check the args
        elif isinstance(statment, ast_inter.Assign):
            #TODO keep and enviromenent ...
            return self._refines(self.nextStatement(statmentL), nodeL)
        elif isinstance(statment, ast_inter.While):
            if statment.condition == S.true:
                return self._refines(statment.program.get_label(), nodeL)
            elif isinstance(node, GuardedChoice):
                if any( self.implies(statment.condition, gs.expression) and self._refines(statment.program.get_label(), gs.id) for gs in mode.guarded_states):
                    pass
                else:
                    return False
                if any( self.implies(Not(statment.condition), gs.expression) and self._refines(self.nextStatement(statmentL), gs.id) for gs in mode.guarded_states):
                    pass
                else:
                    return False
                return True
            else:
                return False
        elif isinstance(statment, ast_inter.If):
            if isinstance(node, GuardedChoice):
                for ifComp in statment.if_list:
                    if any( self.implies(ifComp.condition, gs.expression) and self._refines(ifComp.program.get_label(), gs.id) for gs in mode.guarded_states):
                        pass
                    else:
                        return False
                return True
            else:
                trivial = [ case.program for case in statment.if_list if case.condition == S.true ]
                return any(self._refines(s.get_label(), nodeL) for s in trivial)
        elif isinstance(statment, ast_inter.Send):
            return isinstance(node, SendMessage) and statment.comp == node.receiver and self.sameMsgName(statment.msg_type, node.msg_type) #TODO check the args
        elif isinstance(statment, ast_inter.Receive):
            if not isinstance(node, ExternalChoice):
                return False
            else:
                def findOne(ns):
                    return self._refines(statment.motion.get_label(), ns) or any(self._refines(r.get_label(), ns) for r in statment.actions)
                return all( findOne(ns) for ns in node.end_state )
        elif isinstance(statment, ast_inter.Action):
            return isinstance(node, ReceiveMessage) and statment.str_msg_type == node.msg_type #TODO check the args
        elif isinstance(statment, ast_inter.Print) or isinstance(statment, ast_inter.Skip):
            return self._refines(self.nextStatement(statmentL), nodeL)
        elif isinstance(statment, ast_inter.Statement):
            return self._refines(statment.children[0].get_label(), nodeL)
        else:
            raise Exception("unexpected: " + statment)

    def check(self):
        allLabels = set(self.programLabels.keys())
        allState = set(self.state_to_node.keys())
        self.compat = { l: copy(allState) for l in allLabels }
        changed = True
        while changed:
            changed = False
            for l in allLabels:
                for s in copy(self.compat[l]):
                    if not self.compatible(l, s):
                        if self.debug:
                            print("not compatible", l, s)
                        changed = True
                        self.compat[l].discard(s)
        return self.projection.start_state in self.compat[self.program.get_label()]
