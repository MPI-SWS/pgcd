from sympy import And
from sympy.vector import CoordSys3D

def distance(p1, p2):
    """distance between two points"""
    return p1.position_wrt(p2).magnitude()

def sphere(frame, radius, p):
    """point in a sphere next around the origin"""
    return distance(p, frame.origin) <= radius

def halfSpace(frame, direction, p):
    v = p.position_wrt(frame.origin)
    return v.dot(direction) > 0

def halfSphere(frame, radius, direction, p):
    return And(sphere(frame, radius, p), halfSpace(frame, direction, p))

def cylinder(frame, radius, height, p):
    v = p.position_wrt(frame.origin)
    proj = frame.k.projection(v, scalar=True)
    return And(proj <= height, (v - frame.k.projection(v)).magnitude() <= radius)
