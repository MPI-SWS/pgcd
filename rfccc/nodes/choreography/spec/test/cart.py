from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec import *
from geometry import *

# cart
class Cart(Process):

    def __init__(self, name, world, index = 0):
        super().__init__(name, world, index)
        # dimensions
        self.height = 0.1
        self.radius = 0.2
        # variables
        self._x = symbols(name + '_x')
        self._y = symbols(name + '_y')
        self._theta = symbols(name + '_theta')
        f = world.frame()
        self._position = self._x * f.i + self._y * f.j
        # mount is 11cm above the ground
        self._mount = f.orient_new_axis(name + '_mount', self._theta, f.k, location= self._position + 0.11 * f.k)
        # motion primitives
        MoveFromTo(self)
        Idle(self)

    def position(self):
        f = self.frame()
        return f.orient_new_axis(self._name + '_position', self._theta, f.k, location = self._position)

    def frame(self):
        # reuse the parent's position
        return self._parent.frame()

    def invariant(self):
        rng = 0.01 #bound on the space in which the cart travel around the origin FIXME dreal get slow...
        return And(self._theta >= -mp.pi, self._theta <= mp.pi, self._x <= rng, self._x >= -rng, self._y <= rng, self._y >= -rng)

    def outputVariables(self):
        return [self._x, self._y, self._theta]

    def mountingPoint(self, index):
        assert(index == 0)
        return self._mount

    def ownResources(self, point):
        return cylinder(self.position(), self.radius, self.height, point)

    def abstractResources(self, point, deltaXY = 0.0, deltaXYZ = 0.0):
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
        self._maxError = 0.1

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
        near = distance(self._component.position(), self._src) <= self._maxError
        return And(onGround, near)

    def post(self):
        onGround = self._onGround(self._component.position())
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxError, self._maxError, 0.0)
        return And(onGround, workSpace)

    def inv(self):
        onGround = self._onGround(self._component.position())
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxError)
        return self.timify(And(onGround, workSpace))

    def preFP(self, point):
        return self._component.abstractResources(point, self._maxError, 0.005)

    def postFP(self, point):
        return self._component.abstractResources(point, self._maxError, 0.005)

    def invFP(self, point):
        dst_height = self._dst.locate_new('dsth', self._frame.k * self._height)
        i = cube(self._frame, self._src, dst_height, point, self._maxError + self._radius, self._maxError + self._radius, 0.005)
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
