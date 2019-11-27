import interpreter.ast_inter as ast_inter

class CFA():

    def __init__(self, program, debug = False):
        self.program = program
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
            ls2 = self.buildCFA(lastLabel, statment.motion)
            for l2 in ls2:
                self.nextLabel[l2].add(l)
            lastLabel = { l for i in statment.actions for l in self.buildCFA(lastLabel, i) }
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

    def hasLabel(self, label):
        return label in self.programLabels

    def getStatemnt(self, label):
        return self.programLabels[label]

    def nextStatement(self, label):
        n = self.nextLabel[label]
        if len(n) == 0:
            #raise Exception("no successors for " + str(statment))
            return None
        elif len(n) > 1:
            raise Exception("ambiguous successors: " + n + " for " + str(label))
        else:
            for elt in n:
                return elt
