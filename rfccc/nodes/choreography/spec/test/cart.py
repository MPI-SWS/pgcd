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
        MoveToward(self)
    
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


class MoveToward(MotionPrimitiveFactory):
    
    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return [Symbol('targetLoc')]
    
    def setParameters(self, *args):
        assert(len(args) == 1)
        return MoveTowardLoc(self._component, args[0])


class MoveTowardLoc(MotionPrimitive):
    
    def __init__(self, component, loc):
        super().__init__(component)
        self._frame = self._component.frame()
        self._loc = loc
        self._locVec = loc.position_wrt(self._frame)

    def pre(self):
        return Eq(self._component.frame().k.dot(self._locVec) , 0)
    
    def post(self):
        return distance(self._component.position(), self._loc) <= 0.5
    
    #TODO need to make the formula dependent on the "old" state
    def inv(self):
        return True
    
    def preFP(self, point):
        return True
    
    def postFP(self, point):
        return True
    
    def invFP(self, point):
        return True
