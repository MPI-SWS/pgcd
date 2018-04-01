from sympy import And, Abs, simplify
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

def cube(frame, lowerBackLeft, upperFrontRight, p, maxError = 0.0):
    (px,py,pz) = p.express_coordinates(frame)
    (lx,ly,lz) = lowerBackLeft.express_coordinates(frame)
    (ux,uy,uz) = upperFrontRight.express_coordinates(frame)
    dx = (ux - lx)/2
    mx = lx + dx
    inX = And(px - mx <= Abs(dx) + maxError, px - mx >= -Abs(dx) - maxError)
    dy = (uy - ly)/2
    my = ly + dy
    inY = And(py - my <= Abs(dy) + maxError, py - my >= -Abs(dy) - maxError)
    dz = (uz - lz)/2
    mz = lz + dz
    inZ = And(pz - mz <= Abs(dz) + maxError, pz - mz >= -Abs(dz) - maxError)
    return And(inX, inY, inZ)

def halfSpace(frame, normal, p, maxError = 0.0):
    #plane goes through frame.origin
    #normal must be an unit vector
    v = p.position_wrt(frame.origin)
    return normal.dot(v) >= maxError
