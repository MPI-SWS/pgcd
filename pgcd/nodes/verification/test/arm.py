from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from verification.spec.component import *
from verification.spec.motion import *
from verification.spec.time import *
from verification.utils.geometry import *
import verification.utils.transition
import verification.spec.conf as conf

# model for a generic arm as 3 linkage connected by revolute actuators
class Arm(Process):

    def __init__(self, name, parent, index = 0,
                 a_ref = 0, b_ref = 0, c_ref = 0,
                 useDegree = True):
        super().__init__(name, parent, index)
        if useDegree:
            self.coeff = mp.pi() / 180.0
        else:
            self.coeff = 1.0
        # default dimensions (m, rad)
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
        self.a_ref = self.coeff * a_ref
        self.b_ref = self.coeff * b_ref
        self.c_ref = self.coeff * c_ref
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
        retractArm(self)
        idle(self)
        wait(self)
        rotate(self)
        openGripper(self)
        closeGripper(self)
        setAngleTurntable(self)
        setAngleCantilever(self)
        setAngleAnchorPoint(self)
        Grab(self)
        PutInBin(self)
        MoveTo(self)
        Grip(self)
        Fold(self)
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

class ArmMP(MotionPrimitive):

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

class Fold(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0 or len(args) == 1 or len(args) == 3 or len(args) == 4)
        if len(args) == 0:
            return ArmFold(self.name(), self._component)
        elif len(args) == 1:
            return ArmFold(self.name(), self._component, args[0])
        elif len(args) == 3:
            return ArmSetAllAngles(self.name(), self._component, 
                                   args[0], args[1], args[2], # c, b, a
                                   0, self._component.minAngleAB, self._component.maxAngleAB)
        else:
            return ArmSetAllAngles(self.name(), self._component,
                                   args[0], args[1], args[2],
                                   0, self._component.minAngleAB, self._component.maxAngleAB,
                                   dt = DurationSpec(args[3],args[3],False))

class retractArm(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ['[opt] duration']

    def setParameters(self, args):
        assert(len(args) == 0 or len(args) == 1 or len(args) == 3 or len(args) == 4)
        if len(args) == 0:
            return ArmFold(self.name(), self._component)
        elif len(args) == 1:
            return ArmFold(self.name(), self._component, args[0])
        elif len(args) == 3:
            return ArmSetAllAngles(self.name(), self._component, 
                                   args[0], args[1], args[2], # c, b, a
                                   0, self._component.minAngleAB, self._component.maxAngleAB)
        else:
            return ArmSetAllAngles(self.name(), self._component,
                                   args[0], args[1], args[2],
                                   0, self._component.minAngleAB, self._component.maxAngleAB,
                                   dt = DurationSpec(args[3],args[3],False))

class ArmFold(ArmMP):

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

class idle(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmIdle(self.name(), self._component)

class ArmIdle(ArmMP):

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

class wait(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 1)
        return ArmWait(self.name(), self._component, args[0])

class ArmWait(ArmMP):

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

class Grab(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["target"]

    def setParameters(self, args):
        assert(len(args) in {1,3})
        if len(args) == 1:
            return ArmGrab(self.name(), self._component, args[0])
        else:
            return ArmGrab(self.name(), self._component, args[0], args[1], args[2])

class ArmGrab(ArmMP):

    def __init__(self, name, component, target, pos = None, orientation = None):
        super().__init__(name, component)
        self._target = target
        self._pos = pos
        self._orientation = orientation

    def duration(self):
        return DurationSpec(0, 1, False) #TODO

    def mountA(self):
        assert self._pos != None
        (x1, y1, z1) = self._component.frame().origin.express_coordinates(conf.worldFrame)
        (x2, y2, z2) = self._pos.express_coordinates(conf.worldFrame)
        return ((x1,x2), (y1,y2), (Symbol('C_theta'), self._orientation)) #FIXME hack

    def removeParentVar(self, f):
        if self._pos != None:
            ((x1,x2), (y1,y2), (t1,t2)) = self.mountA()
            f2 = f.subs({x1:x2, y1:y2, t1: t2})
            #print("remove parent", (x1,x2), (y1,y2), (t1,t2))
            #print(f)
            #print(f2)
            return f2
        else:
            print("cannot remove parent var")
            return f

    def targetAngle(self):
        (x, y, z) = self._target.express_coordinates(self._component.frame())
        return self.removeParentVar(atan2(y, x))

    def posCstr(self):
        if self._pos != None:
            ((x1,x2), (y1,y2), (t1,t2)) = self.mountA()
            return And(Eq(x1,x2), Eq(y1,y2), Eq(t1,t2))
        else:
            return S.true

    def preA(self):
        return self.posCstr()

    def preG(self):
        maxRadius = self._component.upperArmLength + self._component.lowerArmLength + self._component.upperArmRadius + self._component.gripperReach
        #TODO min distance
        dist = distance(self._component._upper.origin, self._target) <= maxRadius
        return self.removeParentVar(dist)

    def preA(self):
        return self.timify(self.posCstr())

    def invG(self):
        return S.true

    def postA(self):
        return self.posCstr()

    def postG(self):
        a = S.true #TODO
        b = S.true #TODO
        c = Eq(self._component.c_eff(), self.targetAngle())
        return self.removeParentVar(And(a, b, c))

class PutInBin(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["bin location"]

    def setParameters(self, args):
        assert(len(args) == 1)
        return ArmPutInBin(self.name(), self._component, args[0])

# assume one bin in front then move to the loc and then move back
class ArmPutInBin(ArmMP):

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
        withoutGamma = self._component.abstractResources(point, self.err)
        f = self._component.frame()
        gMin = halfSpace(f,  f.j, point, self._component.baseRadius + self.err)
        gMax = halfSpace(f, -f.j, point, self._component.baseRadius + self.err)
        return And(withoutGamma, gMin, gMax)

    def postFP(self, point):
        withoutGamma = self._component.abstractResources(point, self.err)
        f = self._component.frame()
        gMin = halfSpace(f,  f.j, point, self._component.baseRadius + self.err)
        gMax = halfSpace(f, -f.j, point, self._component.baseRadius + self.err)
        return And(withoutGamma, gMin, gMax)

    def invFP(self, point):
        withoutGamma = self._component.abstractResources(point, self.err)
        f = self._component.frame()
        gMin = halfSpace(f,  f.orient_new_axis("minGammaBound", self._minGamma, f.k).j, point, self._component.baseRadius + self.err)
        gMax = halfSpace(f, -f.orient_new_axis("maxGammaBound", self._maxGamma, f.k).j, point, self._component.baseRadius + self.err)
        return self.timify(And(withoutGamma, gMin, gMax))

# since we don't precisely model the gripper, it is like idle
class closeGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 0)
        return ArmWait(self.name(), self._component, 2)

# since we don't precisely model the gripper, it is like waiting
class openGripper(MotionPrimitiveFactory):

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

class ArmMoveTo(ArmMP):

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

class setAngleTurntable(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle", "[opt] duration"]

    def setParameters(self, args):
        assert(len(args) == 2 or len(args) == 3)
        if len(args) == 2:
            return ArmSetAngle(self.name(), self._component, self._component._c, args[0], args[1])
        else:
            return ArmSetAngle(self.name(), self._component, self._component._c, args[0], args[1], args[2])

class setAngleCantilever(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle", "[opt] duration"]

    def setParameters(self, args):
        assert(len(args) == 2 or len(args) == 3)
        if len(args) == 2:
            return ArmSetAngle(self.name(), self._component, self._component._b, args[0], args[1])
        else:
            return ArmSetAngle(self.name(), self._component, self._component._b, args[0], args[1], args[2])

class setAngleAnchorPoint(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["source angle", "target angle", "[opt] duration"]

    def setParameters(self, args):
        assert(len(args) == 2 or len(args) == 3)
        if len(args) == 2:
            return ArmSetAngle(self.name(), self._component, self._component._a, args[0], args[1])
        else:
            return ArmSetAngle(self.name(), self._component, self._component._a, args[0], args[1], args[2])

class ArmSetAngle(ArmMP):

    def __init__(self, name, component, var, angle1, angle2, dt = None):
        super().__init__(name, component)
        self.var = var
        self.angle1 = self._component.coeff * angle1
        self.angle2 = self._component.coeff * angle2
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

class RotateAndGrab(MotionPrimitiveFactory):

    def parameters(self):
        return ["source turntable", "source cantilever", "source anchor",
                "target turntable", "target cantilever", "target anchor"]

    def setParameters(self, args):
        assert(len(args) in {6, 7})
        if len(args) == 6:
            return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0.35)
        else:
            return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0.35, DurationSpec(args[6], args[6], False))

class RotateAndPut(MotionPrimitiveFactory):

    def parameters(self):
        return ["source turntable", "source cantilever", "source anchor",
                "target turntable", "target cantilever", "target anchor"]

    def setParameters(self, args):
        assert(len(args) in {6, 7})
        if len(args) == 6:
            return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0.35)
        else:
            return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0.35, DurationSpec(args[6], args[6], False))

class rotate(MotionPrimitiveFactory):

    def parameters(self):
        return ["source turntable", "source cantilever", "source anchor",
                "target turntable", "target cantilever", "target anchor"]

    def setParameters(self, args):
        assert(len(args) in {6, 7})
        if len(args) == 6:
            return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0)
        else:
            return ArmSetAllAngles(self.name(), self._component, args[0], args[1], args[2], args[3], args[4], args[5], 0, DurationSpec(args[6], args[6], False))

class ArmSetAllAngles(ArmMP):

    def __init__(self, name, component, angle1a, angle2a, angle3a, angle1b, angle2b, angle3b,
                 delta = 0.0, dt = DurationSpec(0, 1, False), smooth = True):
        super().__init__(name, component)
        self.angle1a = self._component.coeff * angle1a
        self.angle2a = self._component.coeff * angle2a
        self.angle3a = self._component.coeff * angle3a
        self.angle1b = self._component.coeff * angle1b
        self.angle2b = self._component.coeff * angle2b
        self.angle3b = self._component.coeff * angle3b
        self.delta   = self._component.coeff * delta
        self.dt = dt
        self.smooth = smooth

    def modifies(self):
        return self._component.internalVariables()

    def duration(self):
        return self.dt

    def preG(self):
        return And( self._component._a >= self.angle3a - self.err, self._component._a <= self.angle3a + self.err,
                    self._component._b >= self.angle2a - self.err, self._component._b <= self.angle2a + self.err,
                    self._component._c >= self.angle1a - self.err, self._component._c <= self.angle1a + self.err)

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
        if self.smooth:
            t = timeSymbol()
            dt = self.dt.max
            ta = Eq(self._component._a, verification.utils.transition.linear(t, self.angle3a, self.angle3b, dt))
            tb = Eq(self._component._b, verification.utils.transition.linear(t, self.angle2a, self.angle2b, dt))
            tc = Eq(self._component._c, verification.utils.transition.linear(t, self.angle1a, self.angle1b, dt))
            f = And(t >= 0, t <= dt, ta, tb, tc)
        else:
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
