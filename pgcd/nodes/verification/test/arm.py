from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec import *
from utils.geometry import *

# model for a generic arm as 3 linkage connected by revolute actuators
class Arm(Process):

    def __init__(self, name, parent, index = 0):
        super().__init__(name, parent, index)
        # default dimensions
        self.baseHeight = 0.23
        self.baseRadius = 0.13
        self.upperArmLength = 0.22
        self.upperArmRadius = 0.08
        self.lowerArmLength = 0.10
        self.lowerArmRadius = 0.04
        self.gripperReach = 0.08
        self.maxAngleAB =  13*mp.pi()/18
        self.minAngleAB = -13*mp.pi()/18
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
        Grip(self)
        SetAngleTurntable(self)
        SetAngleCantilever(self)
        SetAngleAnchorPoint(self)
        RetractArm(self)
        Rotate(self)
        RotateAndGrab(self)
        RotateAndPut(self)
        PutOnCart(self)
        GetFromCart(self)

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

    def ownResources(self, point, maxError = 0.0):
        baseFP = cylinder(self._base, self.baseRadius, self.baseHeight, point, maxError)
        upperFP = cylinder(self._upper, self.upperArmRadius, self.upperArmLength, point, maxError)
        lowerFP = cylinder(self._lower, self.lowerArmRadius, self.lowerArmLength, point, maxError)
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
        domain_a = And(self._a >= self.minAngleAB, self._a <= self.maxAngleAB)
        domain_b = And(self._b >= self.minAngleAB, self._b <= self.maxAngleAB)
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


class RetractArm(MotionPrimitiveFactory):

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
        a = Eq(self._component.alpha(), self._component.maxAngleAB)
        b = Eq(self._component.beta(), self._component.minAngleAB)
        c = Eq(self._component.gamma(), 0)
        return And(a, b, c)

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
        f = And(self._component.gamma() >= self._minGamma, self._component.gamma() <= self._maxGamma)
        return self.timify(f)

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

# since we don't precisely model the gripper, it is like idle
class Grip(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 1)
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
        return self.timify(pos <= dist)

    def preFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def postFP(self, point):
        return self._component.abstractResources(point, 0.05)

    def invFP(self, point):
        i = self._component.abstractResources(point, 0.05)
        return self.timify(i)

class SetAngleTurntable(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle"]

    def setParameters(self, args):
        assert(len(args) == 2)
        return ArmSetAngle(self.name(), self._component, self._component._c, args[0], args[1])

class SetAngleCantilever(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle"]

    def setParameters(self, args):
        assert(len(args) == 2)
        return ArmSetAngle(self.name(), self._component, self._component._b, args[0], args[1])

class SetAngleAnchorPoint(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle"]

    def setParameters(self, args):
        assert(len(args) == 2)
        return ArmSetAngle(self.name(), self._component, self._component._a, args[0], args[1])

class ArmSetAngle(MotionPrimitive):

    def __init__(self, name, component, var, angle1, angle2):
        super().__init__(name, component)
        self.var = var
        self.angle1 = angle1
        self.angle2 = angle2

    def modifies(self):
        return [self.var, Symbol(self._component.name() + '_dummy')]

    def pre(self):
        return And(self.var >= self.angle1 - 0.01, self.angle1 <= self.angle1 + 0.01 ) #deal with δ-sat

    def post(self):
        return Eq(self.var, self.angle2 )

    def inv(self):
        f = true
        if self.angle1 < self.angle2:
            f = And( self.var >= self.angle1, self.var <= self.angle2)
        else:
            f = And( self.var >= self.angle2, self.var <= self.angle1)
        return self.timify(f)

    def preFP(self, point):
        return self._component.ownResources(point, 0.05)

    def postFP(self, point):
        return self._component.ownResources(point, 0.05)

    def invFP(self, point):
        i = self._component.ownResources(point, 0.05)
        return self.timify(i)

class RotateAndGrab(MotionPrimitiveFactory):

    def parameters(self):
        return ["source turntable", "source cantilever", "source anchor",
                "target turntable", "target cantilever", "target anchor"]

    def setParameters(self, args):
        assert(len(args) == 6)
        return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0.35)

class RotateAndPut(MotionPrimitiveFactory):

    def parameters(self):
        return ["source turntable", "source cantilever", "source anchor",
                "target turntable", "target cantilever", "target anchor"]

    def setParameters(self, args):
        assert(len(args) == 6)
        return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0.35)

class Rotate(MotionPrimitiveFactory):

    def parameters(self):
        return ["source turntable", "source cantilever", "source anchor",
                "target turntable", "target cantilever", "target anchor"]

    def setParameters(self, args):
        assert(len(args) == 6)
        return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0.0)

class ArmSetAllAngles(MotionPrimitive):

    def __init__(self, name, component, angle1a, angle2a, angle3a, angle1b, angle2b, angle3b, delta):
        super().__init__(name, component)
        self.angle1a = angle1a
        self.angle2a = angle2a
        self.angle3a = angle3a
        self.angle1b = angle1b
        self.angle2b = angle2b
        self.angle3b = angle3b
        self.delta   = delta

    def modifies(self):
        return self._component.internalVariables()

    def pre(self):
        return And( self._component._a >= self.angle3a - 0.01, self._component._a <= self.angle3a + 0.01,
                    self._component._b >= self.angle2a - 0.01, self._component._b <= self.angle2a + 0.01,
                    self._component._c >= self.angle1a - 0.01, self._component._c <= self.angle1a + 0.01)

    def post(self):
        targetCanti = self.angle2b
        if targetCanti <= 0.0:
            targetCanti = targetCanti + self.delta
        else:
            targetCanti = targetCanti - self.delta
        return And( Eq( self._component._a, self.angle3b ),
                    Eq( self._component._b, targetCanti ),
                    Eq( self._component._c, self.angle1b ))

    def inv(self):
        f = true
        if self.angle1a < self.angle1b:
            f = And(f, self._component._c >= self.angle1a, self._component._c <= self.angle1b)
        else:
            f = And(f, self._component._c >= self.angle1b, self._component._c <= self.angle1a)
        if self.angle2a < self.angle2b:
            f = And(f, self._component._b >= self.angle2a, self._component._b <= self.angle2b)
        else:
            f = And(f, self._component._b >= self.angle2b, self._component._b <= self.angle2a)
        if self.angle3a < self.angle3b:
            f = And(f, self._component._a >= self.angle3a, self._component._a <= self.angle3b)
        else:
            f = And(f, self._component._a >= self.angle3b, self._component._a <= self.angle3a)
        return self.timify(f)

    def preFP(self, point):
        return self._component.ownResources(point, 0.05)

    def postFP(self, point):
        return self._component.ownResources(point, 0.05)

    def invFP(self, point):
        i = self._component.ownResources(point, 0.05)
        return self.timify(i)

#TODO
class PutOnCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmIdle(self.name(), self._component)

#TODO
class GetFromCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmIdle(self.name(), self._component)
