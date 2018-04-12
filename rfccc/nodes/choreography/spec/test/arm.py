from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec import *
from geometry import *

# model for a generic arm as 3 linkage connected by revolute actuators
class Arm(Process):

    def __init__(self, name, parent, index = 0):
        super().__init__(name, parent, index)
        # default dimensions
        self.baseHeight = 0.1
        self.baseRadius = 0.15
        self.upperArmLength = 0.2
        self.upperArmRadius = 0.05
        self.lowerArmLength = 0.05
        self.lowerArmRadius = 0.05
        self.gripperReach = 0.5
        # variables
        self._a = symbols(name + '_a') # rotation along the Y-axis (upper arm to lower arm)
        self._b = symbols(name + '_b') # rotation along the Y-axis (base to upper arm)
        self._c = symbols(name + '_c') # rotation along the Z-axis (base)
        #TODO state of the gripper
        # frame and stuff
        self._frame = parent.mountingPoint(index)
        self._base = self._frame.orient_new_axis(name + '_base', self._c, self._frame.k)
        self._upper = self._base.orient_new_axis(name + '_upper', self._b, self._base.j, location= self.baseHeight * self._base.k)
        self._lower = self._upper.orient_new_axis(name + '_lower', self._a, self._upper.j, location= self.upperArmLength * self._upper.k)
        self._effector = self._lower.locate_new(name + '_effector', self.lowerArmLength * self._lower.k)
        # motion primitives
        Fold(self)
        Idle(self)
        Grab(self)
        PutInBin(self)
        MoveTo(self)
        OpenGripper(self)
        CloseGripper(self)

    def frame(self):
        return self._frame

    def alpha(self):
        '''rotation along the Y-axis (upper arm to lower arm)'''
        return self._a

    def beta(self):
        '''rotation along the Y-axis (base to upper arm)'''
        return self._b

    def gamma(self):
        '''rotation along the Z-axis (base)'''
        return self._c

    def internalVariables(self):
        return [self._a, self._b, self._c, Symbol(self._name + '_dummy')]

    def ownResources(self, point):
        baseFP = cylinder(self._base, self.baseRadius, self.baseHeight, point)
        upperFP = cylinder(self._upper, self.upperArmRadius, self.upperArmLength, point)
        lowerFP = cylinder(self._lower, self.lowerArmRadius, self.lowerArmLength, point)
        return Or(baseFP, upperFP, lowerFP)

    def abstractResources(self, point, delta = 0.0):
        f = self.frame()
        # that one is very abstract
        # r = self.baseHeight + self.upperArmLength + self.lowerArmLength + self.upperArmRadius
        r1 = sqrt( self.baseHeight**2 + self.upperArmLength**2 - 2 * self.baseHeight * self.upperArmLength * cos(mp.pi - self._b))
        r2 = sqrt( r1**2 + self.lowerArmLength**2 - 2 * r1 * self.lowerArmLength * cos(mp.pi - self._a))
        r = r2 + self.upperArmRadius + delta
        return halfSphere(f, r, f.k, point)

    def mountingPoint(self, index):
        assert(index == 0)
        return self._effector

    def invariant(self):
        pi = mp.pi
        domain_a = And(self._a >= -pi/2, self._a <= pi/2)
        domain_b = And(self._b >= 0, self._b <= 2*pi/3)
        domain_c = And(self._c >= -pi, self._c <= pi)
        return And(domain_a, domain_b, domain_c)

class Fold(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmFold(self.name(), self._component)


class ArmFold(MotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)

    def pre(self):
        return S.true

    def post(self):
        a = Eq(self._component.alpha(), mp.pi/2)
        b = Eq(self._component.beta(), mp.pi/2)
        return And(a, b)

    def inv(self):
        return S.true

    def preFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def postFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def invFP(self, point):
        i = self._component.abstractResources(point, 0.05)
        return self.timify(i)

class Idle(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmIdle(self.name(), self._component)

class ArmIdle(MotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)

    def modifies(self):
        return [Symbol(self._component.name() + '_dummy')]

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

class Grab(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["target"]

    def setParameters(self, args):
        assert(len(args) == 1)
        return ArmGrab(self.name(), self._component, args[0])

class ArmGrab(MotionPrimitive):

    def __init__(self, name, component, target):
        super().__init__(name, component)
        self._target = target

    def pre(self):
        maxRadius = self._component.upperArmLength + self._component.lowerArmLength + self._component.upperArmRadius + self._component.gripperReach
        #TODO min distance
        return distance(self._component._upper.origin, self._target) <= maxRadius

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

class PutInBin(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["bin location"]

    def setParameters(self, args):
        assert(len(args) == 1)
        return ArmPutInBin(self.name(), self._component, args[0])

# assume one bin in front then move to the loc and then move back
class ArmPutInBin(MotionPrimitive):

    def __init__(self, name, component, target):
        super().__init__(name, component)
        self._target = target
        (x, y, z) = target.express_coordinates(component.frame())
        angle = atan2(y, x)
        self._minGamma = Min(angle, 0).evalf()
        self._maxGamma = Max(angle, 0).evalf()

    def neutral(self, maxError = 0.0):
        if maxError <= 0.0:
            a = Eq(self._component.alpha(), mp.pi/2)
            b = Eq(self._component.beta(), mp.pi/2)
            c = Eq(self._component.gamma(), 0.0)
            return And(a,b,c)
        else:
            a = And(self._component.alpha() >= mp.pi/2 - maxError, self._component.alpha() <= mp.pi/2 + maxError)
            b = And(self._component.beta() >= mp.pi/2 - maxError, self._component.beta() <= mp.pi/2 + maxError)
            c = And(self._component.gamma() >= -maxError, self._component.gamma() <= maxError)
            return And(a,b,c)

    def pre(self):
        maxRadius = self._component.upperArmLength + self._component.lowerArmLength + self._component.gripperReach
        maxDist = distance(self._component._upper.origin, self._target) <= maxRadius
        #TODO min distance
        return And(self.neutral(0.1), maxDist)

    def post(self):
        return self.neutral()

    def inv(self):
        return And(self._component.gamma() >= self._minGamma, self._component.gamma() <= self._maxGamma)

    def preFP(self, point):
        withoutGamma = self._component.abstractResources(point, 0.05)
        f = self._component.frame()
        gMin = halfSpace(f,  f.j, point, self._component.baseRadius + 0.05)
        gMax = halfSpace(f, -f.j, point, self._component.baseRadius + 0.05)
        return And(withoutGamma, gMin, gMax)

    def postFP(self, point):
        withoutGamma = self._component.abstractResources(point, 0.05)
        f = self._component.frame()
        gMin = halfSpace(f,  f.j, point, self._component.baseRadius + 0.05)
        gMax = halfSpace(f, -f.j, point, self._component.baseRadius + 0.05)
        return And(withoutGamma, gMin, gMax)

    def invFP(self, point):
        withoutGamma = self._component.abstractResources(point, 0.05)
        f = self._component.frame()
        gMin = halfSpace(f,  f.orient_new_axis("minGammaBound", self._minGamma, f.k).j, point, self._component.baseRadius + 0.05)
        gMax = halfSpace(f, -f.orient_new_axis("maxGammaBound", self._maxGamma, f.k).j, point, self._component.baseRadius + 0.05)
        return self.timify(And(withoutGamma, gMin, gMax))

# since we don't precisely model the gripper, it is like idle
class CloseGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmIdle(self.name(), self._component)

# since we don't precisely model the gripper, it is like idle
class OpenGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmIdle(self.name(), self._component)

class MoveTo(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["target"]

    def setParameters(self, args):
        assert(len(args) == 1)
        return ArmMoveTo(self.name(), self._component, args[0])

class ArmMoveTo(MotionPrimitive):

    def __init__(self, name, component, target):
        super().__init__(name, component)
        self._target = target

    def pre(self):
        maxRadius = self._component.upperArmLength + self._component.lowerArmLength + self._component.gripperReach
        #TODO min distance
        dist = distance(self._component._upper.origin, self._target)
        pos = distance(self._component._upper.origin, self._component.mountingPoint(0).origin)
        return And(pos <= dist, dist <= maxRadius)

    def post(self):
        effector = self._component.mountingPoint(0).origin
        return distance(effector, self._target) <= 0.01

    def inv(self):
        dist = distance(self._component._upper.origin, self._target)
        pos = distance(self._component._upper.origin, self._component.mountingPoint(0).origin)
        return pos <= dist

    def preFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def postFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def invFP(self, point):
        i = self._component.abstractResources(point, 0.05)
        return self.timify(i)
