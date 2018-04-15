from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec import *
from geometry import *

#TODO the 2nd cart of a cube 0.18 wide, 0.17 long, 0.16 high
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
        # variables
        self._x = symbols(name + '_x')
        self._y = symbols(name + '_y')
        self._theta = symbols(name + '_theta')
        f = world.frame()
        self._position = self._x * f.i + self._y * f.j
        # mount is 11cm above the ground
        self._mount = f.orient_new_axis(name + '_mount', self._theta, f.k, location= self._position + 0.09 * f.k)
        # motion primitives
        MoveFromTo(self)
        Idle(self)
        MoveCart(self)
        StrafeCart(self)
        SetAngleCart(self)

    def position(self):
        f = self.frame()
        return f.orient_new_axis(self._name + '_position', self._theta, f.k, location = self._position)

    def frame(self):
        # reuse the parent's position
        return self._parent.frame()

    def invariant(self):
        rng = 10 #bound on the space in which the cart travel around the origin, FIXME dreal does not like unbounded variables
        return And(self._theta >= -mp.pi, self._theta <= mp.pi, self._x <= rng, self._x >= -rng, self._y <= rng, self._y >= -rng)

    def outputVariables(self):
        return [self._x, self._y, self._theta]

    def mountingPoint(self, index):
        assert(index == 0)
        return self._mount

    def ownResources(self, point):
        return semiRegularHexagon(self.position(), self.minRadius, self.maxRadius, self.height, point)

    def abstractResources(self, point, deltaXY = 0.01, deltaXYZ = 0.01):
        return cylinder(self.position(), self.radius + deltaXY + deltaXYZ, self.height + deltaXYZ, point)

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

    def ownResources(self, point):
        pos = self.position()
        o = pos.origin
        return cube(pos, o.locate_new(-l2 * pos.i - w2 * pos.j), o.locate_new(l2 * pos.i + w2 * pos.j + self.height * pos.k), p)

    def abstractResources(self, point, deltaXY = 0.01, deltaXYZ = 0.01):
        return cylinder(self.position(), self.radius + deltaXY + deltaXYZ, self.height + deltaXYZ, point)

class MoveFromTo(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['source', 'target']

    def setParameters(self, args):
        assert(len(args) == 2)
        return CartMove(self.name(), self._component, args[0], args[1])


class CartMove(MotionPrimitive):

    def __init__(self, name, component, src, dst):
        super().__init__(name, component)
        self._frame = self._component.frame()
        self._radius = component.radius
        self._height = component.height
        self._src = src
        self._dst = dst
        self._maxErrorPre = 0.1
        self._maxErrorPost = 0.01

    def _locAsVec(self, loc):
        return loc.position_wrt(self._frame)

    def _onGround(self, loc):
        return Eq(self._component.frame().k.dot(self._locAsVec(loc)) , 0)

    def _srcFrame(self):
        return self._frame.locate_new("src", self._locAsVec(self._src))

    def _dstFrame(self):
        return self._frame.locate_new("dst", self._locAsVec(self._dst))

    def pre(self):
        onGround = And(self._onGround(self._src), self._onGround(self._dst), self._onGround(self._component.position()))
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxErrorPre, self._maxErrorPre, 0.0)
        return And(onGround, workSpace)

    def post(self):
        onGround = self._onGround(self._component.position())
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxErrorPost, self._maxErrorPost, 0.0)
        return And(onGround, workSpace)

    def inv(self):
        onGround = self._onGround(self._component.position())
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxErrorPost, self._maxErrorPost, 0.0)
        return self.timify(And(onGround, workSpace))

    def preFP(self, point):
        return self._component.abstractResources(point, self._maxErrorPre, 0.005)

    def postFP(self, point):
        return self._component.abstractResources(point, self._maxErrorPost, 0.005)

    def invFP(self, point):
        dst_height = self._dst.locate_new('dsth', self._frame.k * self._height)
        i = cube(self._frame, self._src, dst_height, point, self._maxErrorPost + self._radius, self._maxErrorPost + self._radius, 0.005)
        return self.timify(i)

class Idle(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return CartIdle(self.name(), self._component)

class CartIdle(MotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)

    def modifies(self):
        return []

    def pre(self):
        return S.true

    def post(self):
        return S.true

    def inv(self):
        return S.true

    def preFP(self, point):
        return self._component.abstractResources(point, 0.0, 0.005)

    def postFP(self, point):
        return self._component.abstractResources(point, 0.0, 0.005)

    def invFP(self, point):
        i = self._component.abstractResources(point, 0.0, 0.005)
        return self.timify(i)

class SetAngleCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["target angle"]

    def setParameters(self, args):
        assert(len(args) == 1)
        return CartSetAngle(self.name(), self._component, args[0])

class CartSetAngle(MotionPrimitive):

    def __init__(self, name, component, angle):
        super().__init__(name, component)
        self.var = self._component._theta
        self.angle = angle

    def modifies(self):
        return [self.var]

    def pre(self):
        return S.true

    def post(self):
        return Eq(self.var, self.angle)

    def inv(self):
        return S.true

    def preFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def postFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def invFP(self, point):
        i = self._component.abstractResources(point, 0.05)
        return self.timify(i)

class MoveCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['x', 'y', 't', 'delta']

    def setParameters(self, args):
        assert(len(args) == 4)
        direction = self._component.position().i * args[3]
        return CartMoveDirection(self.name(), self._component, args[0], args[1], args[2], direction)

class StrafeCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['x', 'y', 't', 'delta']

    def setParameters(self, args):
        assert(len(args) == 4)
        direction = self._component.position().j * args[3]
        return CartMoveDirection(self.name(), self._component, args[0], args[1], args[2], direction)
        
class CartMoveDirection(MotionPrimitive):
    
    def __init__(self, name, component, x, y, t, direction):
        super().__init__(name, component)
        self._frame = self._component.frame()
        self.x = x
        self.y = y
        self.t = t
        self.d = direction
        self._maxErrorPre = 0.01
        self._maxErrorPost = 0.005
        self._radius = component.radius
        self._height = component.height

    def _locAsVec(self, loc):
        return loc.position_wrt(self._frame)

    def _srcFrame(self):
        return self._frame.locate_new("src", self._frame.i * self.x + self._frame.j * self.y)

    def _dstFrame(self):
        return self._frame.locate_new("dst", self._frame.i * self.x + self._frame.j * self.y + self.d)
    
    def _onGroundVec(self, vec):
        return Eq(self._component.frame().k.dot(vec) , 0)
    
    def _onGround(self, loc):
        return Eq(self._component.frame().k.dot(self._locAsVec(loc)) , 0)

    def modifies(self):
        return [self._component._x, self._component._y]

    def pre(self):
        onGround = And(self._onGroundVec(self.d), self._onGround(self._component.position()))
        workSpace = distance(self._component.position().origin, self._srcFrame().origin) <= 0.01 #because δ-sat
        return And(onGround, workSpace)

    def post(self):
        onGround = self._onGround(self._component.position())
        workSpace = Eq(distance(self._component.position().origin, self._dstFrame().origin), 0.0)
        return And(onGround, workSpace)

    def inv(self):
        onGround = self._onGround(self._component.position())
        #proj of position on direction is 0
        workSpace = cube(self._frame, self._srcFrame().origin, self._dstFrame().origin, self._component.position().origin, self._maxErrorPost, self._maxErrorPost, 0.0)
        return self.timify(And(onGround, workSpace))

    def preFP(self, point):
        return self._component.abstractResources(point, self._maxErrorPre, 0.005)

    def postFP(self, point):
        return self._component.abstractResources(point, self._maxErrorPost, 0.005)

    def invFP(self, point):
        dst_height = self._dstFrame().origin.locate_new('dsth', self._frame.k * self._height)
        i = cube(self._frame, self._srcFrame().origin, dst_height, point, self._maxErrorPost + self._radius, self._maxErrorPost + self._radius, 0.005)
        return self.timify(i)
