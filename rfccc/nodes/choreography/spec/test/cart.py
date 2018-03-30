from sympy import *
from sympy.vector import CoordSys3D
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
    
    def position(self):
        f = self.frame()
        return f.orient_new_axis(self._name + '_position', self._theta, f.k, location = self._position)

    def frame(self):
        # reuse the parent's position
        return self._parent.frame()
    
    def outputVariables(self):
        return [self._x, self._y, self._theta]
    
    def mountingPoint(self, index):
        assert(index == 0)
        return self._mount
    
    def ownResources(self, point):
        return cylinder(self.position(), self.radius, self.height, point)


class MoveFromTo(MotionPrimitiveFactory):
    
    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['sourceX', 'sourceY','targetX','targetY']
    
    def setParameters(self, *args):
        assert(len(args) == 4)
        return CartMove(self._component, args[0], args[1], args[2], args[3])


class CartMove(MotionPrimitive):
    
    def __init__(self, component, srcX, srcY, dstX, dstY):
        super().__init__(component)
        self._frame = self._component.frame()
        self._radius = component.radius
        self._height = component.height
        self._src = self._frame.origin.locate_new('src', srcX * self._frame.i + srcY * self._frame.j)
        self._dst = self._frame.origin.locate_new('dst', dstX * self._frame.i + dstY * self._frame.j)
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
        near = distance(self._component.position(), self._dst) <= self._maxError
        return And(onGround, near)
    
    def inv(self):
        onGround = self._onGround(self._component.position())
        workSpace = cube(self._frame, self._src, self._dst, self._component.position().origin, self._maxError)
        return self.timify(And(onGround, workSpace))
    
    def preFP(self, point):
        return cylinder(self._srcFrame(), self._radius + self._maxError, self._height, point)
    
    def postFP(self, point):
        return cylinder(self._dstFrame(), self._radius + self._maxError, self._height, point)
    
    def invFP(self, point):
        dst_height = self._dst.locate_new('dsth', self._frame.k * self._height)
        pos_as_point = self._component.position().origin
        i = cube(self._frame, self._src, dst_height, pos_as_point, self._maxError + self._radius)
        return self.timify(i)

#TODO idle motion primitive
#the tricky part is that it maintain the ruccent state, so it is state-dependent
