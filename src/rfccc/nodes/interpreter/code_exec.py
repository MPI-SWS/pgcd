from parser import *
from abc import abstractmethod


class Executor:
    variables = {}

    def __init__(self):
        self.parser = RoboParser()

    def execute(self, code):
        self.parser.parse(code).accept(self)

    @abstractmethod
    def visit_send(self, component, label, expression):
        pass

    @abstractmethod
    def visit_receive(self, motion, actions):
        pass

    @abstractmethod
    def do_action(self, value, exps):
        pass

    @abstractmethod
    def attribute(self, name):
        pass

    def visit(self, node):

        if node.tip == Type.statement:
            for stmt in node.children:
                stmt.accept(self)

        elif node.tip == Type.skip:
            pass

        elif node.tip == Type.send:
            self.visit_send(node.comp, node.label, node.message.accept(self))
            # print('sending ', str(node.comp), str(node.label), str(node.message.accept(self)))

        elif node.tip == Type.receive:
            self.visit_receive(node.motion, [{"msg": a.message, "var": a.id_name, "stmt": a.stmt} for a in node.actions])
            # print('receiving')

        elif node.tip == Type.action:
            return

        elif node.tip == Type._if:
            cond = node.condition.accept(self)
            if cond:
                node.ifCode.accept(self)
            else:
                node.elseCode.accept(self)

        elif node.tip == Type._while:
            cond = node.condition.accept(self)
            while cond:
                node.code.accept(self)
                cond = node.condition.accept(self)

        elif node.tip == Type.assign:
            exp = node.value.accept(self)
            if node.id_Prop is None:
                if isinstance(exp, dict):
                    self.variables[node.id] = exp
                else:
                    if not self.variables.__contains__(node.id):
                        self.variables[node.id] = {}
                    self.variables[node.id]['value'] = exp
            else:
                self.variables[node.id][node.id_Prop.value] = exp

        elif node.tip == Type._tuple:
            dic = {}
            for key, value in node.tup.items():
                k = key.accept(self)
                v = value.accept(self)
                dic[k] = v
            return dic

        elif isinstance(node, BinOp):
            if node.tip == Type.dot:
                if node.exp1 in self.variables.keys():
                    return self.variables[node.exp1][node.exp2]
                else:
                    return self.attribute(node.exp1).__getattribute__(node.exp2)
            var1 = node.exp1.accept(self)
            var2 = node.exp2.accept(self)
            if isinstance(var1, str) or isinstance(var2, str):
                var1 = '"' + str(var1) + '"'
                var2 = '"' + str(var2) + '"'
            return eval(str(var1) + ' ' + str(node.sign) + ' ' + str(var2))

        elif isinstance(node, UnOp):
            if node.tip == Type.id:
                if node.exp in self.variables.keys():
                    return self.variables[node.exp]
                else:
                    return self.attribute(node.exp)
            if node.tip == Type._print:
                for item in node.exp:
                    var1 = item.accept(self)
                    print(var1)
                return
            var1 = node.exp.accept(self)
            return eval(str(node.sign) + ' (' + str(var1) + ') ')

        elif isinstance(node, Constant):
            return node.value

        elif isinstance(node, Motion):
            args = [x.accept(self) for x in node.exps]
            self.do_action(node.value, args)
