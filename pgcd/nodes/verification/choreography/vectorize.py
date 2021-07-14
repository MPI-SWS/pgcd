# replace the variables with vectors in the spec
from verification.choreography.ast_chor import *
from verification.utils.vectorizer import *

def vectorize(choreography, world):
    '''replace variables by vectors in the world's frame'''
    #interpret function Vec(x,y,z) as constructor for vectors
    #interpret function Pnt(x,y,z) as constructor for point
    #interpret id_x/y/z as accessing coord
    #an id get converted to a point (id_x, id_y, id_z) in the world's frame
    known = { v for p in world.allProcesses() for v in p.variables() }
    vec = Vectorizer(world.frame(), known)

    for node in choreography.statements:
        if isinstance(node, Message):
            expressions2 = [ vec.apply(e) for e in node.expressions ]
            node.expressions = expressions2
        elif isinstance(node, GuardedChoice):
            for gs in node.guarded_states:
                expr2 = vec.apply(gs.expression)
                gs.expression = expr2
        elif isinstance(node, Motion):
            for motion in node.motions:
                motion.mp_args = [ vec.apply(arg) for arg in motion.mp_args ]
        else:
            pass
