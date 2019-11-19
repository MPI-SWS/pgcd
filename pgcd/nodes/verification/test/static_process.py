from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec import *
from utils.geometry import *

# a model static things that executes code like sensors.
# they have a footprint (a cube), execute code, but cannot move.


# cart
class StaticProcess(Process):

    def __init__(self, name, x, y, z, theta, dx, dy, dz, parent = None, index = 0):
        super().__init__(name, parent, index)
        self.x = x
        self.y = y
        self.z = z
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.theta = theta
        self.dummyVar = Symbol(name + '_dummy')
        Idle(self)
        Wait(self)
    
    def frame(self):
        return self._parent.frame() #by default use the parent's frame

    def internalVariables(self):
        return [self.dummyVar]
    
    def footprint(self, point):
        f = self.frame()
        cf = f.orient_new_axis(self.name(), self.theta, f.k, location = self.x * f.i + self.y * f.j + self.z * f.k)
        return cube(cubeFrame, cf.origin, cf.origin.locate_new(self.dx * cf.i + self.dy * cf.j + self.dz * cf.k) , point)
    
    def mountingPoint(self, index):
        return ValueException(self.name() + " does not have mounting moints.")

class Idle(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return StaticIdle(self.name(), self._component)

class StaticIdle(MotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)

    def modifies(self):
        return [self._component.dummyVar]
    
    def duration(self):
        return DurationSpec(0, float('inf'), True) 

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

class Wait(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        if len(args) == 1:
            return StaticWait(self.name(), self._component, args[0])
        elif len(args) == 2:
            return StaticWait(self.name(), self._component, args[0], args[1])
        else:
            assert Fasle, "wrong args " + str(args)

class StaticWait(MotionPrimitive):

    def __init__(self, name, component, t_min, t_max = -1):
        super().__init__(name, component)
        self.t_min = t_min
        if t_max < 0:
            self.t_max = t_min
        else:
            self.t_max = t_max

    def modifies(self):
        return [self._component.dummyVar]
    
    def duration(self):
        return DurationSpec(self.t_min, self.t_max, False)

    def pre(self):
        return S.true

    def post(self):
        return S.true

    def inv(self):
        return S.true

    def preFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def postFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def invFP(self, point):
        i = self._component.abstractResources(point, 0.05)
        return self.timify(i)
