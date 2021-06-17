from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec.component import *
from spec.motion import *
from spec.time import *
import spec.conf
from utils.geometry import *
import utils.transition

# model for a 3-axis cartesian motion platform
class Crane(Process):
    
    def __init__(self, name, parent, index = 0):
        super().__init__(name, parent, index)
        # variables
        self._x = symbols(name + '_x')
        self._y = symbols(name + '_y')
        self._z = symbols(name + '_z')
        #TODO physical dimension for footprints
        self.maxX = ???
        self.maxY = ???
        self.maxZ = ???
        # frame and stuff
        self._frame = parent.mountingPoint(index)
        self._base = self._frame.locate_new(name + '_base', ??? )
        self._upper = self._base.locate_new(name + '_upper', ??? )
        self._lower = self._upper.locate_new(name + '_lower', ??? )
        self._effector = self._lower.locate_new(name + '_effector', ??? )
        # motion primitives
        Home(self)
        MoveTo(self)
        MoveToX(self)
        MoveToY(self)
        MoveToZ(self)
        MoveToXY(self)
        OpenGripper(self)
        CloseGripper(self)
        Grip(self)
        Idle(self)
        Wait(self)
        MoveObject(self)
    

    def internalVariables(self):
        return [self._x, self._y, self._z, Symbol(self._name + '_dummy')]

    def ownResources(self, point, maxError = 0.0):
        baseFP = ???
        upperFP = ???
        lowerFP = ???
        return Or(baseFP, upperFP, lowerFP)

    def abstractResources(self, point, delta = 0.0):
        f = self.frame()
        pass

    def mountingPoint(self, index):
        assert(index == 0)
        return self._effector

    def invariantG(self):
        domain_x = And(self._x >= 0, self._x <= self.maxX)
        domain_y = And(self._y >= 0, self._y <= self.maxY)
        domain_z = And(self._z >= 0, self._z <= self.maxZ)
        return And(domain_x, domain_y, domain_z)


class CraneMP(MotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)
        self.err = 0.01

    def preFP(self, point):
        #return self._component.abstractResources(point, self.err)
        return self._component.ownResources(point, self.err)

    def postFP(self, point):
        #return self._component.abstractResources(point, self.err)
        return self._component.ownResources(point, self.err)

    def invFP(self, point):
        #i = self._component.abstractResources(point, self.err)
        i = self._component.ownResources(point, self.err)
        return self.timify(i)


class Idle(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return CraneIdle(self.name(), self._component)

class CraneIdle(CraneMP):

    def __init__(self, name, component):
        super().__init__(name, component)

    def modifies(self):
        return [Symbol(self._component.name() + '_dummy')]

    def duration(self):
        return DurationSpec(0, float('inf'), True) 

    def preG(self):
        return S.true

    def postG(self):
        return S.true

    def invG(self):
        return S.true


class Wait(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 1)
        return CraneWait(self.name(), self._component, args[0])

class CraneWait(CraneMP):

    def __init__(self, name, component, t_min, t_max = -1):
        super().__init__(name, component)
        self.t_min = t_min
        if t_max < 0:
            self.t_max = t_min
        else:
            self.t_max = t_max

    def modifies(self):
        return [Symbol(self._component.name() + '_dummy')]
    
    def duration(self):
        return DurationSpec(self.t_min, self.t_max, False)

    def preG(self):
        return S.true

    def postG(self):
        return S.true

    def invG(self):
        return S.true


class Home(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return CraneHome(self.name(), self._component)

class CraneHome(CraneMP):

    def __init__(self, name, component):
        super().__init__(name, component)

    def modifies(self):
        return [self._component._x, self._component._y, self._component._z]

    def duration(self):
        return DurationSpec(1, 2, False)

    def preG(self):
        return S.true

    def postG(self):
        x = Eq(self._component._x, 0)
        y = Eq(self._component._y, 0)
        z = Eq(self._component._z, 0)
        return And(x, y, z)

    def invG(self):
        return S.true


class MoveTo(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 3)
        return CraneMoveTo(self.name(), self._component, args[0], args[1], args[2])

class CraneMoveTo(CraneMP):

    def __init__(self, name, component, x, y, z):
        super().__init__(name, component)
        self.x = x
        self.y = y
        self.z = z

    def modifies(self):
        return [self._component._x, self._component._y, self._component._z]

    def duration(self):
        return DurationSpec(0, 3, False)

    def preG(self):
        return S.true

    def postG(self):
        x = Eq(self._component._x, self.x)
        y = Eq(self._component._y, self.y)
        z = Eq(self._component._z, self.z)
        return And(x, y, z)

    def invG(self):
        return S.true


class MoveToX(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 1)
        return CraneMoveToX(self.name(), self._component, args[0])

class CraneMoveToX(CraneMP):

    def __init__(self, name, component, x):
        super().__init__(name, component)
        self.x = x

    def modifies(self):
        return [self._component._x]

    def duration(self):
        return DurationSpec(0, 1, False)

    def preG(self):
        return S.true

    def postG(self):
        return Eq(self._component._x, self.x)

    def invG(self):
        return S.true


class MoveToY(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 1)
        return CraneMoveToY(self.name(), self._component, args[0])

class CraneMoveToY(CraneMP):

    def __init__(self, name, component, y):
        super().__init__(name, component)
        self.y = y

    def modifies(self):
        return [self._component._y]

    def duration(self):
        return DurationSpec(0, 1, False)

    def preG(self):
        return S.true

    def postG(self):
        return Eq(self._component._y, self.y)

    def invG(self):
        return S.true


class MoveToZ(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 1)
        return CraneMoveToZ(self.name(), self._component, args[0])

class CraneMoveToZ(CraneMP):

    def __init__(self, name, component, z):
        super().__init__(name, component)
        self.z = z

    def modifies(self):
        return [self._component._z]

    def duration(self):
        return DurationSpec(0, 2, False)

    def preG(self):
        return S.true

    def postG(self):
        return Eq(self._component._z, self.z)

    def invG(self):
        return S.true


class MoveToXY(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 2)
        return CraneMoveToZ(self.name(), self._component, args[0], args[1])

class CraneMoveToXY(CraneMP):

    def __init__(self, name, component, x, y):
        super().__init__(name, component)
        self.x = x
        self.y = y

    def modifies(self):
        return [self._component._x, self._component._y]

    def duration(self):
        return DurationSpec(0, 2, False)

    def preG(self):
        return S.true

    def postG(self):
        x = Eq(self._component._x, self.x)
        y = Eq(self._component._y, self.y)
        return And(x, y)

    def invG(self):
        return S.true

class OpenGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return CraneWait(self.name(), self._component, 1)

class CloseGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return CraneWait(self.name(), self._component, 1)

class Grip(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return CraneWait(self.name(), self._component, 1)


class MoveObject(MotionPrimitiveFactory)

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 6)
        return CraneMoveObject(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5])

class CraneMoveObject(CraneMP):

    def __init__(self, name, component, x0, y0, z0, x1, y1, z1):
        super().__init__(name, component)
        self.x0 = x0
        self.x0 = x0
        self.y0 = y0
        self.z1 = z1
        self.y1 = y1
        self.z1 = z1

    def modifies(self):
        return [self._component._x, self._component._y, self._component._z]

    def duration(self):
        return DurationSpec(0, 5, False)

    def preG(self):
        return S.true

    def postG(self):
        x = Eq(self._component._x, self.x1)
        y = Eq(self._component._y, self.y1)
        z = Eq(self._component._z, 0)
        return And(x, y, z)

    def invG(self):
        part1 = And(Eq(self._component._x,self.x0), Eq(self._component._y,self.y0), self._component._z >= 0, self._component._z <= self.z0)
        part2 = And( Eq(self._component._z, 0)) #TODO x, y
        part3 = And(Eq(self._component._x,self.x1), Eq(self._component._y,self.y1), self._component._z >= 0, self._component._z <= self.z1)
        return Or(part1, part2, part3)
    
