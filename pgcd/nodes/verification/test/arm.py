from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from spec.component import *
from spec.motion import *
from spec.time import *
from utils.geometry import *

# model for a generic arm as 3 linkage connected by revolute actuators
class Arm(Process):

    def __init__(self, name, parent, index = 0, a_ref = 0, b_ref = 0, c_ref = 0):
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
        # where if the 0 compared to the neutra joint position
        self.a_ref = a_ref
        self.b_ref = b_ref
        self.c_ref = c_ref
        #TODO angular speed
        # variables
        self._a = symbols(name + '_a') # rotation along the Y-axis (upper arm to lower arm)
        self._b = symbols(name + '_b') # rotation along the Y-axis (base to upper arm)
        self._c = symbols(name + '_c') # rotation along the Z-axis (base)
        #TODO state of the gripper
        # frame and stuff
        self._frame = parent.mountingPoint(index)
        self._base = self._frame.orient_new_axis(name + '_base',    self._c + self.c_ref, self._frame.k)
        self._upper = self._base.orient_new_axis(name + '_upper',   self._b + self.b_ref, self._base.j, location= self.baseHeight * self._base.k)
        self._lower = self._upper.orient_new_axis(name + '_lower',  self._a + self.a_ref, self._upper.j, location= self.upperArmLength * self._upper.k)
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

    def a_eff(self):
        return self._a + self.a_ref

    def b_eff(self):
        return self._b + self.b_ref

    def c_eff(self):
        return self._c + self.c_ref

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
        r1 = sqrt( self.baseHeight**2 + self.upperArmLength**2 - 2 * self.baseHeight * self.upperArmLength * cos(mp.pi - self.b_eff()))
        r2 = sqrt( r1**2 + self.lowerArmLength**2 - 2 * r1 * self.lowerArmLength * cos(mp.pi - self.a_eff()))
        r = r2 + self.upperArmRadius + delta
        return halfSphere(f, r, f.k, point)

    def mountingPoint(self, index):
        assert(index == 0)
        return self._effector

    def invariantG(self):
        pi = mp.pi
        domain_a = And(self.a_eff() >= self.minAngleAB, self.a_eff() <= self.maxAngleAB)
        domain_b = And(self.b_eff() >= self.minAngleAB, self.b_eff() <= self.maxAngleAB)
        domain_c = And(self.c_eff() >= -pi, self.c_eff() <= pi)
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
        return ['[opt] duration']

    def setParameters(self, args):
        assert(len(args) == 0 or len(args) == 1)
        if (len(args) == 0):
            return ArmFold(self.name(), self._component)
        else:
            return ArmFold(self.name(), self._component, args[0])

class ArmFold(MotionPrimitive):

    def __init__(self, name, component, duration = None):
        super().__init__(name, component)
        if duration is None:
            self._dMin = 0
            self._dMax = 1
        else:
            self._dMin = duration
            self._dMax = duration
    
    def duration(self):
        return DurationSpec(self._dMin, self._dMax, False) #TODO function of angle and speed

    def preG(self):
        return S.true

    def postG(self):
        a = Eq(self._component.a_eff(), self._component.maxAngleAB)
        b = Eq(self._component.b_eff(), self._component.minAngleAB)
        c = Eq(self._component.c_eff(), 0)
        return And(a, b, c)

    def invG(self):
        return S.true

    def preFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def postFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def invFP(self, point):
        #i = self._component.abstractResources(point, 0.05)
        i = self._component.ownResources(point, 0.05)
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

    def duration(self):
        return DurationSpec(0, float('inf'), True) 

    def preG(self):
        return S.true

    def postG(self):
        return S.true

    def invG(self):
        return S.true

    def preFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def postFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def invFP(self, point):
        #i = self._component.abstractResources(point, 0.05)
        i = self._component.ownResources(point, 0.05)
        return self.timify(i)

class ArmWait(MotionPrimitive):

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

    def preFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def postFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def invFP(self, point):
        #i = self._component.abstractResources(point, 0.05)
        i = self._component.ownResources(point, 0.05)
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

    def duration(self):
        return DurationSpec(0, 1, False) #TODO 

    def preG(self):
        maxRadius = self._component.upperArmLength + self._component.lowerArmLength + self._component.upperArmRadius + self._component.gripperReach
        #TODO min distance
        return distance(self._component._upper.origin, self._target) <= maxRadius

    def postG(self):
        return S.true

    def invG(self):
        return S.true

    def preFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def postFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def invFP(self, point):
        #i = self._component.abstractResources(point, 0.05)
        i = self._component.ownResources(point, 0.05)
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
            a = Eq(self._component.a_eff(), mp.pi/2)
            b = Eq(self._component.b_eff(), mp.pi/2)
            c = Eq(self._component.c_eff(), 0.0)
            return And(a,b,c)
        else:
            a = And(self._component.a_eff() >= mp.pi/2 - maxError, self._component.a_eff() <= mp.pi/2 + maxError)
            b = And(self._component.b_eff() >= mp.pi/2 - maxError, self._component.b_eff() <= mp.pi/2 + maxError)
            c = And(self._component.c_eff() >= -maxError, self._component.c_eff() <= maxError)
            return And(a,b,c)

    def duration(self):
        return DurationSpec(0, 1, False) #TODO 

    def preG(self):
        maxRadius = self._component.upperArmLength + self._component.lowerArmLength + self._component.gripperReach
        maxDist = distance(self._component._upper.origin, self._target) <= maxRadius
        #TODO min distance
        return And(self.neutral(0.1), maxDist)

    def postG(self):
        return self.neutral()

    def invG(self):
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
        return ArmWait(self.name(), self._component, 2)

# since we don't precisely model the gripper, it is like waiting
class OpenGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmWait(self.name(), self._component, 2)

# since we don't precisely model the gripper, it is like waiting
class Grip(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) <= 1)
        return ArmWait(self.name(), self._component, 2)

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

    def duration(self):
        return DurationSpec(0, 1, False) #TODO function of angle and speed

    def preG(self):
        maxRadius = self._component.upperArmLength + self._component.lowerArmLength + self._component.gripperReach
        #TODO min distance
        dist = distance(self._component._upper.origin, self._target) # can reach
        return And(dist <= maxRadius)

    def postG(self):
        effector = self._component.mountingPoint(0).origin
        return distance(effector, self._target) <= 0.01

    def invG(self):
        #dist = distance(self._component._upper.origin, self._target)
        #pos = distance(self._component._upper.origin, self._component.mountingPoint(0).origin)
        #return self.timify(pos <= dist)
        return S.true

    def preFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def postFP(self, point):
        #return self._component.abstractResources(point, 0.05)
        return self._component.ownResources(point, 0.05)

    def invFP(self, point):
        #i = self._component.abstractResources(point, 0.05)
        i = self._component.ownResources(point, 0.05)
        return self.timify(i)

class SetAngleTurntable(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle", "[opt] duration"]

    def setParameters(self, args):
        assert(len(args) == 2 or len(args) == 3)
        if len(args) == 2:
            return ArmSetAngle(self.name(), self._component, self._component.c_eff(), args[0], args[1])
        else:
            return ArmSetAngle(self.name(), self._component, self._component.c_eff(), args[0], args[1], args[2])

class SetAngleCantilever(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle", "[opt] duration"]

    def setParameters(self, args):
        assert(len(args) == 2 or len(args) == 3)
        if len(args) == 2:
            return ArmSetAngle(self.name(), self._component, self._component.b_eff(), args[0], args[1])
        else:
            return ArmSetAngle(self.name(), self._component, self._component.b_eff(), args[0], args[1], args[2])

class SetAngleAnchorPoint(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle", "[opt] duration"]

    def setParameters(self, args):
        assert(len(args) == 2 or len(args) == 3)
        if len(args) == 2:
            return ArmSetAngle(self.name(), self._component, self._component.a_eff(), args[0], args[1])
        else:
            return ArmSetAngle(self.name(), self._component, self._component.a_eff(), args[0], args[1], args[2])

class ArmSetAngle(MotionPrimitive):

    def __init__(self, name, component, var, angle1, angle2, dt = None):
        super().__init__(name, component)
        self.var = var
        self.angle1 = angle1
        self.angle2 = angle2
        if dt is None:
            self._dMin = 0
            self._dMax = 1
        else:
            self._dMin = dt
            self._dMax = dt

    def modifies(self):
        return [self.var, Symbol(self._component.name() + '_dummy')]

    def duration(self):
        return DurationSpec(self._dMin, self._dMax, False) #TODO function of angle and speed

    def preG(self):
        return And(self.var >= self.angle1 - 0.01, self.angle1 <= self.angle1 + 0.01 ) #deal with δ-sat

    def postG(self):
        return Eq(self.var, self.angle2 )

    def invG(self):
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

    def duration(self):
        return DurationSpec(0, 1, False) #TODO function of angle and speed

    def preG(self):
        return And( self._component._a >= self.angle3a - 0.01, self._component._a <= self.angle3a + 0.01,
                    self._component._b >= self.angle2a - 0.01, self._component._b <= self.angle2a + 0.01,
                    self._component._c >= self.angle1a - 0.01, self._component._c <= self.angle1a + 0.01)

    def postG(self):
        targetCanti = self.angle2b
        if targetCanti <= 0.0:
            targetCanti = targetCanti + self.delta
        else:
            targetCanti = targetCanti - self.delta
        a = Eq( self._component._a, self.angle3b )
        b = Eq( self._component._b, targetCanti )
        c = Eq( self._component._c, self.angle1b )
        return And(a, b, c)

    def invG(self):
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
        return ArmWait(self.name(), self._component, 10)

#TODO
class GetFromCart(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmWait(self.name(), self._component, 10)
