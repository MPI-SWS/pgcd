from subprocess import Popen, PIPE, run, CalledProcessError, TimeoutExpired
import tempfile
import os

from typing import List, Dict, Any, cast, Tuple

from arpeggio import Optional, ZeroOrMore, OneOrMore, EOF
from arpeggio import Parser, RegExMatch, ParserPython, NoMatch
from arpeggio import PTNodeVisitor, visit_parse_tree

from sympy.core import Expr, Add, Mul, Pow, Symbol, Integer, sympify, Float
from sympy.core.compatibility import default_sort_key, string_types
from sympy.core.function import _coeff_isneg
from sympy.core.mul import _keep_coeff
from sympy.core.relational import Eq, Ne, Lt, Le, Gt, Ge
from sympy.polys.polytools import poly
from sympy.printing.precedence import precedence, PRECEDENCE
from sympy.printing.printer import Printer
from sympy import And, Or, QQ

class DrealPrinter(Printer):
    printmethod = "_dreal"

    def _print_BooleanTrue(self, expr):
        return "true"

    def _print_BooleanFalse(self, expr):
        return "false"

    def _print_Add(self, expr):
        args = Add.make_args(expr)
        if len(args) == 1:
            return self._print(args[0])
        else:
            return "(+ " + " ".join([ self._print(arg) for arg in args ]) + ")"

    def _print_Mul(self, expr):
        c, e = expr.as_coeff_Mul()
        if c < 0:
            expr = _keep_coeff(-c, e)
            sign_pre = "(- "
            sign_post = ")"
        else:
            sign_pre = ""
            sign_post = ""
        args = Mul.make_args(expr)
        if len(args) == 1:
            return sign_pre + self._print(args[0]) + sign_post
        else:
            return sign_pre + "(* " + " ".join([ self._print(arg) for arg in args ]) + ")" + sign_post
    
    def _print_And(self, expr):
        return "(and " + " ".join([ self._print(arg) for arg in expr.args ]) + ")"

    def _print_Or(self, expr):
        return "(or " + " ".join([ self._print(arg) for arg in expr.args ]) + ")"

    def _print_Not(self, expr):
        return "(not " + " ".join([ self._print(arg) for arg in expr.args ]) + ")"

    def _print_Abs(self, expr):
        return "(abs " + " ".join([ self._print(arg) for arg in expr.args ]) + ")"

    def _print_Function(self, expr):
        return "(" + str(expr.func) + " " + " ".join([ self._print(arg) for arg in expr.args ]) + ")"

    def _print_Relational(self, expr):
        charmap_pre = {
            "==": "=",
            "!=": "not (="
        }
        charmap_post = {
            "!=": ")"
        }
        prec = precedence(expr)
        return '(%s %s %s %s)' % (charmap_pre.get(expr.rel_op) or expr.rel_op, self._print(expr.lhs), self._print(expr.rhs), charmap_post.get(expr.rel_op) or "")

    def _print_Pow(self, expr):
        prec = precedence(expr)
        return '(^ %s %s)' % (self._print(expr.base), self._print(expr.exp))

    def _print_Poly(self, expr):
        e = expr.as_expr()
        return self._print(e)

    def _print_Rational(self, expr):
        if expr.q == 1:
            return self._print(expr.p)
        else:
            return '(/ %s %s)' % (self._print(expr.p), self._print(expr.q))

class DrealParseTreeVisitor(PTNodeVisitor):

    def visit_ident(self, node, children):
        return Symbol(node.value)

    def visit_number(self, node, children):
        return Float(node.value)

    def visit_variable(self, node, children):
        return (children[0], children[1], children[2])

    def visit_model(self, node, children):
        m = {}
        for i in range(0, len(children)):
            v, lb, ub = children[i]
            m[v] = (lb, ub)
        return m

    def visit_top(self, node, children):
        if len(children) > 0:
            return children[0]
        else:
            return {}

class DrealInterface:

    def __init__(self, precision: float = 0.001, timeout: int = 120, debug: bool = True) -> None:
        self.precision = precision
        self.timeout = timeout
        self.debug = debug

    # parsing using https://github.com/igordejanovic/Arpeggio
    def parser(self) -> Parser:
        def ident():      return RegExMatch(r"\w+")
        def number():     return RegExMatch(r'[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?')
        def variable():   return ident, ":", "[", number, ",", number, "]"
        def model():      return ZeroOrMore(variable)
        def top():        return model, EOF
        return ParserPython(top) #, debug = self.debug)

    def parseResult(self, string):
        try:
            p = self.parser()
            tree = p.parse(string)
            return visit_parse_tree(tree, DrealParseTreeVisitor())
        except NoMatch as e:
            return None

    def readUntil(self, stream, expected):
        line = stream.readline().strip();
        if self.debug:
            print("< " + line)
        while line != expected:
            line = stream.readline().strip();
            if self.debug:
                print("< " + line)
    
    def write(self, stream, string):
        stream.write(string)
        if self.debug:
            print(string, end='')

    def approximateModel(self, model):
        m2 = {}
        for var, (lb,ub) in model.items():
            m2[var] = (lb+ub) / 2
        return m2

    # https://docs.python.org/3.6/library/subprocess.html
    def run(self, exprs: List[Expr]):
        variables = { v for x in exprs for v in x.free_symbols }
        printer = DrealPrinter()
        command = ["dreal", "--precision", str(self.precision), "--model", "--in"]
        proc = Popen(command, stdin=PIPE, stdout=PIPE, stderr=PIPE, universal_newlines=True)
        # print the model
        self.write(proc.stdin, "(set-logic QF_NRA)\n")
        for var in variables:
            self.write(proc.stdin, "(declare-fun ")
            self.write(proc.stdin, str(var))
            self.write(proc.stdin, " () Real)\n")
        for exp in exprs:
            self.write(proc.stdin, "(assert ")
            stringExp = printer.doprint(exp)
            self.write(proc.stdin, stringExp)
            self.write(proc.stdin, ")\n")
        self.write(proc.stdin, "(check-sat)\n")
        self.write(proc.stdin, "(exit)\n")
        proc.stdin.flush()
        # get the solution
        try:
            outs, errs = proc.communicate(timeout=self.timeout)
            exit_code = proc.wait()
            if exit_code == 0:
                if self.debug:
                    print("< " + outs)
                lines = outs.split("\n")
                if lines[0].startswith("delta-sat"):
                    model = self.parseResult("\n".join(lines[1:]))
                    return (True, model)
                elif lines[0].startswith("unsat"):
                    return (False, None)
                else:
                    s = "\n".join(lines)
                    print( "lines=",lines)
                    raise Exception(s)
            else:
                if self.debug:
                    print("< " + errs)
                raise Exception(errs)
        except TimeoutExpired:
            proc.kill()
            outs, errs = proc.communicate() # for clean-up
            return (None, None)

