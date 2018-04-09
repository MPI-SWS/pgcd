# replace the varaibles with vectors in the spec
from ast_chor import *
from sympy import *

def vectorize(choreography, world):
    '''replace variables by vectors in the world's frame'''
    #interpret function Vec(x,y,z) as constructor for vectors
    #interpret function Pnt(x,y,z) as constructor for point
    #interpret id_x/y/z as accessing coord
    #an id get converted to a point (id_x, id_y, id_z) in the world's frame
    f = world.frame()
    seen = {}

    def isKnown(sym):
        return any([ sym in p.variables() for p in world.allProcesses() ])

    def updateSympyExpr(expr):
        if isinstance(expr, Symbol):
            if isKnown(expr):
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
            elif expr in seen:
                return seen[expr]
            else:
                s = str(expr)
                #print("converting " + s)
                sx = Symbol(s + "_x")
                sy = Symbol(s + "_y")
                sz = Symbol(s + "_z")
                pt = f.origin.locate_new(s, sx*f.i + sy*f.j + sz*f.k)
                seen[expr] = pt
                return pt
        elif isinstance(expr, Function) and str(expr.func) == "Vec":
            assert len(expr.args) == 3
            name = "Vector_"+str(expr.args[0])+"_"+str(expr.args[1])+"_"+str(expr.args[2])
            return f.locate_new(name, expr.args[0] * f.i + expr.args[1] * f.j + expr.args[2] * f.k)
        elif isinstance(expr, Function) and str(expr.func) == "Pnt":
            assert len(expr.args) == 3
            name = "point_"+str(expr.args[0])+"_"+str(expr.args[1])+"_"+str(expr.args[2])
            return f.origin.locate_new(name, expr.args[0] * f.i + expr.args[1] * f.j + expr.args[2] * f.k)
        elif isinstance(expr, AtomicExpr):
            return expr
        elif isinstance(expr, Function) or isinstance(expr, Rel) or isinstance(expr, Expr):
            args2 = tuple( updateSympyExpr(a) for a in expr.args )
            #print(expr)
            #print(expr.func)
            #print(expr.args)
            #print(args2)
            return expr.func(*args2)
        else:
            #print(expr)
            #print(type(expr))
            return expr

    for node in choreography.statements:
        if isinstance(node, Message):
            expressions2 = [ updateSympyExpr(e) for e in node.expressions ]
            node.expressions = expressions2
        elif isinstance(node, GuardedChoice):
            for gs in node.guarded_states:
                expr2 = updateSympyExpr(gs.expression)
                gs.expression = expr2
        elif isinstance(node, Motion):
            for motion in node.motions:
                motion.mp_args = [ updateSympyExpr(arg) for arg in motion.mp_args ]
        else:
            pass
