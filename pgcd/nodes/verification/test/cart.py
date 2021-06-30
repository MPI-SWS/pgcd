from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec.component import *
from spec.motion import *
from spec.time import *
from utils.geometry import *
import utils.transition

#when modeled as triangle then center 2 side is about 0.16

# cart
class Cart(Process):

    def __init__(self, name, world, index = 0):
        super().__init__(name, world, index)
        # dimensions
        self.height = 0.08
        self.radius = 0.30
        self.minRadius = 0.15
        self.maxRadius = 0.25
        self.bedHeight= 0.15 #TODO take FP overapprox into account
        # variables
        self._x = symbols(name + '_x')
        self._y = symbols(name + '_y')
        self._theta = symbols(name + '_theta')
        f = world.mountingPoint(index)
        self._position = self._x * f.i + self._y * f.j
        # mount is 11cm above the ground 
        self._mount = f.orient_new_axis(name + '_mount', self._theta, f.k, location= self._position + self.bedHeight * f.k)
        # motion primitives
        moveFromTo(self)
        idle(self)
        wait(self)
        moveCart(self)
        strafeCart(self)
        SetAngleCart(self)
        swipe(self)

    def position(self):
        f = self.frame()
        return f.orient_new_axis(self._name + '_position', self._theta, f.k, location = self._position)

    def invariantG(self):
        rng = 10 #bound on the space in which the cart travel around the origin, FIXME dreal does not like unbounded variables
        return And(self._theta >= -mp.pi, self._theta <= mp.pi, self._x <= rng, self._x >= -rng, self._y <= rng, self._y >= -rng)

    def outputVariables(self):
        return [self._x, self._y, self._theta]

    def mountingPoint(self, index):
        assert(index == 0)
        return self._mount

    def ownResources(self, point, deltaXY = 0.0, deltaXYZ = 0.0):
        return semiRegularHexagon(self.position(),
                                  self.minRadius+deltaXY+deltaXYZ,
                                  self.maxRadius+deltaXY+deltaXYZ,
                                  self.height+deltaXYZ,
                                  point)

    def abstractResources(self, point, deltaXY = 0.001, deltaXYZ = 0.001):
        return cylinder(self.position(), self.radius + deltaXY + deltaXYZ, self.height + deltaXYZ, point)

#the 2nd cart of a cube 0.18 wide, 0.17 long, 0.16 high
class CartSquare(Cart):

    def __init__(self, name, world, index = 0):
        super().__init__(name, world, index)
        # dimensions
        self.height = 0.16
        self.width = 0.18
        self.length = 0.17
        self.radius = 0.125

    def mountingPoint(self, index):
        assert False

    def ownResources(self, point, deltaXY = 0.0, deltaXYZ = 0.0):
        pos = self.position()
        o = pos.origin
        l2 = self.length / 2 + deltaXY + deltaXYZ
        w2 = self.width / 2 + deltaXY + deltaXYZ
        lowerBackLeft = o.locate_new(self.name() + "_lowerBackLeft", -l2 * pos.i - w2 * pos.j - deltaXYZ * pos.k)
        upperFrontRight = o.locate_new(self.name() + "_upperFrontRight", l2 * pos.i + w2 * pos.j + (self.height+deltaXYZ) * pos.k)
        return cube(pos, lowerBackLeft, upperFrontRight, point)

    def abstractResources(self, point, deltaXY = 0.001, deltaXYZ = 0.001):
        return cylinder(self.position(), self.radius + deltaXY + deltaXYZ, self.height + deltaXYZ, point)


class CartMotionPrimitive(MotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)
        self.err = 0.005
        self._maxErrorPre = 0.005
        self._maxErrorPost = 0.005

    def _locAsVec(self, loc):
        return loc.position_wrt(self._frame)

    def _onGroundVec(self, vec):
        return Eq(self._component.frame().k.dot(vec) , 0)

    def _onGround(self, loc):
        return Eq(self._component.frame().k.dot(self._locAsVec(loc)) , 0)

    def preFP(self, point):
        #return self._component.abstractResources(point, 0.0, self.err)
        return self._component.ownResources(point, 0.0, self.err)

    def postFP(self, point):
        #return self._component.abstractResources(point, 0.0, self.err)
        return self._component.ownResources(point, 0.0, self.err)

    def invFP(self, point):
        #i = self._component.abstractResources(point, 0.0, self.err)
        i = self._component.ownResources(point, 0.0, self.err)
        return self.timify(i)

class moveFromTo(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['source', 'target', '[orientation]']

    def setParameters(self, args):
        assert(len(args) in {2,3})
        if len(args) == 2:
            return CartMove(self.name(), self._component, args[0], args[1])
        else:
            return CartMove(self.name(), self._component, args[0], args[1], args[2])


class CartMove(CartMotionPrimitive):

    def __init__(self, name, component, src, dst, orientation = None):
        super().__init__(name, component)
        self._frame = self._component.frame()
        self._radius = component.radius
        self._height = component.height
        self._src = src
        self._dst = dst
        self._theta = orientation

    def _srcFrame(self):
        return self._frame.locate_new("src", self._locAsVec(self._src))

    def _dstFrame(self):
        return self._frame.locate_new("dst", self._locAsVec(self._dst))

    def orientation(self):
        if self._theta != None:
            return Eq(self._component._theta, self._theta)
        else:
            return S.true

    def duration(self):
        return DurationSpec(0, 1, False) #TODO upper as function of the distance and speed

    def preG(self):
        onGround = And(self._onGround(self._src), self._onGround(self._dst), self._onGround(self._component.position()))
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxErrorPre, self._maxErrorPre, 0.0)
        return And(onGround, workSpace, self.orientation())

    def postG(self):
        onGround = self._onGround(self._component.position())
        #workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxErrorPost, self._maxErrorPost, 0.0)
        workSpace = distance(self._component.position().origin, self._dst) <= 0.0
        return And(onGround, workSpace, self.orientation())

    def invG(self):
        onGround = self._onGround(self._component.position())
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxErrorPost, self._maxErrorPost, 0.0)
        return self.timify(And(onGround, workSpace, self.orientation()))

class idle(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return CartIdle(self.name(), self._component)

class CartIdle(CartMotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)

    def modifies(self):
        return []

    def duration(self):
        return DurationSpec(0, float('inf'), True)

    def preG(self):
        return S.true

    def postG(self):
        return S.true

    def invG(self):
        return S.true

class wait(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        if len(args) == 1:
            return CartWait(self.name(), self._component, args[0])
        elif len(args) == 2:
            return CartWait(self.name(), self._component, args[0], args[1])
        else:
            assert False, "wrong args " + str(args)

class CartWait(CartMotionPrimitive):

    def __init__(self, name, component, t_min, t_max = -1):
        super().__init__(name, component)
        self.t_min = t_min
        if t_max < 0:
            self.t_max = t_min
        else:
            self.t_max = t_max

    def duration(self):
        return DurationSpec(self.t_min, self.t_max, False)
    
    def preG(self):
        return S.true

    def postG(self):
        return S.true

    def invG(self):
        return S.true

class SetAngleCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['[opt] initial angle', "target angle", '[opt] duration']

    def setParameters(self, args):
        assert(len(args) in {1, 2, 3})
        if len(args) == 1:
            return CartSetAngle(self.name(), self._component, args[0])
        elif len(args) == 2:
            return CartSetAngle(self.name(), self._component, args[0], dt = args[1])
        else:
            return CartSetAngle(self.name(), self._component, args[1], angleFrom = args[0], dt = args[2])

class CartSetAngle(CartMotionPrimitive):

    def __init__(self, name, component, angle, angleFrom = None, dt = None):
        super().__init__(name, component)
        self.var = self._component._theta
        self.angle = angle
        if dt is None:
            self._dMin = 0
            self._dMax = 1
        else:
            self._dMin = dt
            self._dMax = dt
        self.af = angleFrom

    def modifies(self):
        return [self.var]

    def duration(self):
        return DurationSpec(self._dMin, self._dMax, False) #TODO upper as function of the angle and angular speed

    def preG(self):
        if self.af != None:
            return And(self.var - self.err <= self.af, self.var <= self.af + self.err)
        else:
            return S.true

    def invG(self):
        if self.af != None:
            t = timeSymbol()
            dt = self.duration().max
            return And( t >= 0, t <= dt, Eq(self.var, utils.transition.linear(t, self.af, self.angle, dt)))
        else:
            return S.true

    def postG(self):
        return Eq(self.var, self.angle)


class moveCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['x', 'y', 't', 'delta', '[opt duration]']

    def setParameters(self, args):
        assert(len(args) == 4 or len(args) == 5)
        direction = self._component.position().i * args[3]
        dx = cos(args[2]) * args[3]
        dy = sin(args[2]) * args[3]
        if len(args) == 4:
            return CartMoveDirection(self.name(), self._component, args[0], args[1], args[2], args[0] + dx, args[1] + dy)
        else:
            return CartMoveDirection(self.name(), self._component, args[0], args[1], args[2], args[0] + dx, args[1] + dy, args[4])


class strafeCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['x', 'y', 't', 'delta']

    def setParameters(self, args):
        assert(len(args) == 4)
        direction = self._component.position().j * args[3]
        dx = sin(args[2]) * args[3]
        dy = cos(args[2]) * args[3]
        return CartMoveDirection(self.name(), self._component, args[0], args[1], args[2], args[0] + dx, args[1] + dy)

class CartMoveDirection(CartMotionPrimitive):

    def __init__(self, name, component, x, y, t, x1, y1, dt = None):
        super().__init__(name, component)
        self._frame = self._component.frame()
        self.x = x
        self.y = y
        self.t = t
        self.x1 = x1
        self.y1 = y1
        if dt is None:
            self._dMin = 3
            self._dMax = 10
        else:
            self._dMin = dt
            self._dMax = dt
        self._radius = component.radius
        self._height = component.height

    def _srcFrame(self):
        return self._frame.locate_new("src", self._frame.i * self.x + self._frame.j * self.y)

    def _dstFrame(self):
        return self._frame.locate_new("dst", self._frame.i * self.x1 + self._frame.j * self.y1)

    def modifies(self):
        return [self._component._x, self._component._y]

    def duration(self):
        return DurationSpec(self._dMin, self._dMax, False) #TODO upper as function of the distance+angle and speed

    def preG(self):
        onGround = self._onGround(self._component.position())
        workSpace = distance(self._component.position().origin, self._srcFrame().origin) <= 0.01 #because δ-sat
        return And(onGround, workSpace)

    def postG(self):
        onGround = self._onGround(self._component.position())
        workSpace = Eq(distance(self._component.position().origin, self._dstFrame().origin), 0.0)
        return And(onGround, workSpace)

    def invG(self):
        onGround = self._onGround(self._component.position())
        pos = self._component.position().origin.position_wrt(self._srcFrame().origin)
        traj = self._dstFrame().origin.position_wrt(self._srcFrame().origin)
        proj = traj.projection(pos, scalar=True)
        return And( proj >= 0, proj <= 1, (pos - traj.projection(pos)).magnitude() <= self.err)


class swipe(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['x', 'y', 't', 'radius', 'angle', '[time]']

    def setParameters(self, args):
        assert(len(args) in {5, 6})
        if len(args) == 5:
            return CartSwipe(self.name(), self._component, args[0], args[1], args[2], args[3], args[4])
        else:
            return CartSwipe(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], DurationSpec(args[5], args[5], False))

class CartSwipe(CartMotionPrimitive):

    def __init__(self, name, component, x, y, t, radius, angle, time = DurationSpec(0, 1, False)):
        super().__init__(name, component)
        self._frame = self._component.frame()
        self.x = x
        self.y = y
        self.t = t
        self.r = radius
        self.a = angle
        self.d = time

    def modifies(self):
        return [self._component._x, self._component._y, self._component._theta]

    def _src(self):
        return self._frame.locate_new("src", self._frame.i * self.x + self._frame.j * self.y).origin

    def centerOfRotation(self):
        f = self._frame
        f2 = f.orient_new_axis("CoR_tmp", self.t, f.k, location = self._src().position_wrt(f.origin))
        return f2.locate_new("CoR", f2.i * self.r)

    def _dst(self):
        cor = self.centerOfRotation()
        rotated = cor.orient_new_axis("rCoR", self.a, cor.k)
        return rotated.origin.locate_new("dst", -rotated.i * self.r)

    def _angleRange(self):
        angleMin = min(self.t + self.a, self.t)
        angleMax = max(self.t + self.a, self.t)
        return And(self._component._theta >= angleMin, self._component._theta <= angleMax)

    def duration(self):
        return self.d

    def preG(self):
        onGround = self._onGround(self._component.position())
        workSpace = distance(self._component.position().origin, self._src()) <= self._maxErrorPre #because δ-sat
        orientation = And(self._component._theta - self.t >= -self.err, self._component._theta - self.t <= self.err)
        return And(onGround, workSpace, orientation)

    def postG(self):
        onGround = self._onGround(self._component.position())
        workSpace = Eq(distance(self._component.position().origin, self._dst()), 0.0)
        angleTarget = Eq(self._component._theta, self.t + self.a)
        return And(onGround, workSpace, angleTarget)

    def invG(self):
        onGround = self._onGround(self._component.position())
        d = distance(self._component.position().origin, self.centerOfRotation().origin)
        workSpace = And(d >= self.r - self._maxErrorPost, d <= self.r + self._maxErrorPost)
        return self.timify(And(onGround, workSpace, self._angleRange()))

#   def invFP(self, point):
#       cor = self.centerOfRotation()
#       v = point.position_wrt(cor.origin)
#       # Z
#       projK = cor.k.projection(v, scalar=True)
#       zRange = And(projK >= 0, projK <= self._component.height)
#       # distance from trajectory radius
#       projIJ = (v - cor.k.projection(v))
#       d = projIJ.magnitude()
#       rRanges = And(d <= self.r + self._component.radius + self._maxErrorPost,
#                     d >= self.r - self._component.radius - self._maxErrorPost)
#       # TODO angle
#       aRange = true
#       # together
#       return self.timify(And(rRanges, zRange, aRange))
