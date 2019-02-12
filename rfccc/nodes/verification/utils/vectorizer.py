# replace the variables with vectors in the spec
from sympy import *

class Vectorizer():

    def __init__(self, frame, dontTouch):
        self.f = frame
        self.known = { v for v in dontTouch }
        self.seen = dict()

    def apply(self, expr):
        if isinstance(expr, Symbol):
            if expr in self.known:
                #print("known " + str(expr))
                return expr
            elif str(expr).endswith("_x"):
                #print("_x " + str(expr))
                return expr
            elif str(expr).endswith("_y"):
                #print("_y " + str(expr))
                return expr
            elif str(expr).endswith("_z"):
                #print("_z " + str(expr))
                return expr
            elif expr in self.seen:
                return self.seen[expr]
            else:
                s = str(expr)
                #print("converting " + s)
                sx = Symbol(s + "_x")
                sy = Symbol(s + "_y")
                sz = Symbol(s + "_z")
                pt = self.f.origin.locate_new(s, sx*self.f.i + sy*self.f.j + sz*self.f.k)
                self.seen[expr] = pt
                return pt
        elif isinstance(expr, Function) and str(expr.func) == "Vec":
            assert len(expr.args) == 3
            name = "Vector_"+str(expr.args[0])+"_"+str(expr.args[1])+"_"+str(expr.args[2])
            return self.f.locate_new(name, expr.args[0] * self.f.i + expr.args[1] * self.f.j + expr.args[2] * self.f.k)
        elif isinstance(expr, Function) and str(expr.func) == "Pnt":
            assert len(expr.args) == 3
            name = "point_"+str(expr.args[0])+"_"+str(expr.args[1])+"_"+str(expr.args[2])
            return self.f.origin.locate_new(name, expr.args[0] * self.f.i + expr.args[1] * self.f.j + expr.args[2] * self.f.k)
        elif isinstance(expr, AtomicExpr):
            return expr
        elif isinstance(expr, Function) or isinstance(expr, Rel) or isinstance(expr, Expr):
            args2 = tuple( self.apply(a) for a in expr.args )
            #print(expr)
            #print(expr.func)
            #print(expr.args)
            #print(args2)
            return expr.func(*args2)
        else:
            #print(expr)
            #print(type(expr))
            return expr
