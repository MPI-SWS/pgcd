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

def cylinder(frame, radius, height, p, maxError = 0.0):
    v = p.position_wrt(frame.origin)
    proj = frame.k.projection(v, scalar=True)
    return And(proj >= -maxError, proj <= height + maxError, (v - frame.k.projection(v)).magnitude() <= radius + maxError)

def cube(frame, lowerBackLeft, upperFrontRight, p, maxErrorX = 0.0, maxErrorY = 0.0, maxErrorZ = 0.0):
    (px,py,pz) = p.express_coordinates(frame)
    (lx,ly,lz) = lowerBackLeft.express_coordinates(frame)
    (ux,uy,uz) = upperFrontRight.express_coordinates(frame)
    dx = (ux - lx)/2
    mx = lx + dx
    inX = And(px - mx <= Abs(dx) + maxErrorX, px - mx >= -Abs(dx) - maxErrorX)
    dy = (uy - ly)/2
    my = ly + dy
    inY = And(py - my <= Abs(dy) + maxErrorY, py - my >= -Abs(dy) - maxErrorY)
    dz = (uz - lz)/2
    mz = lz + dz
    inZ = And(pz - mz <= Abs(dz) + maxErrorZ, pz - mz >= -Abs(dz) - maxErrorZ)
    return And(inX, inY, inZ)

def halfSpace(frame, normal, p, offset = 0.0):
    '''on the "positive" side of the plane which goes through frame.origin and oriented by a normal (unit vector)'''
    v = p.position_wrt(frame.origin)
    return normal.dot(v) >= offset

def triangle(frame, centerToSide, height, p, maxError = 0.0):
    side1 = halfSpace(frame, -0.5 * frame.i - 0.866 * frame.j, p, -centerToSide - maxError)
    side2 = halfSpace(frame, -0.5 * frame.i + 0.866 * frame.j, p, -centerToSide - maxError)
    side3 = halfSpace(frame, frame.i, p, -centerToSide - maxError)
    v = p.position_wrt(frame.origin)
    projZ = frame.k.projection(v, scalar=True)
    z = And(projZ >= -maxError, projZ <= height + maxError)
    return And(side1, side2, side3, z)

def semiRegularHexagon(frame, centerToSide1, centerToSide2, height, p, maxError = 0.0):
    side1a = halfSpace(frame, -0.5 * frame.i - 0.866 * frame.j, p, -centerToSide1 - maxError)
    side1b = halfSpace(frame,  0.5 * frame.i + 0.866 * frame.j, p, -centerToSide2 - maxError)
    side2a = halfSpace(frame, -0.5 * frame.i + 0.866 * frame.j, p, -centerToSide1 - maxError)
    side2b = halfSpace(frame,  0.5 * frame.i - 0.866 * frame.j, p, -centerToSide2 - maxError)
    side3a = halfSpace(frame, frame.i, p, -centerToSide1 - maxError)
    side3b = halfSpace(frame, - frame.i, p, -centerToSide2 - maxError)
    v = p.position_wrt(frame.origin)
    projZ = frame.k.projection(v, scalar=True)
    z = And(projZ >= -maxError, projZ <= height + maxError)
    return And(side1a, side1b, side2a, side2b, side3a, side3b, z)
