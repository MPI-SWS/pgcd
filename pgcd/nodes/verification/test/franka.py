from sympy import *
from sympy.vector import CoordSys3D
from mpmath import mp
from verification.spec.component import *
from verification.spec.motion import *
from verification.spec.time import *
from verification.utils.geometry import *
import verification.utils.transition

# A rough model for a franka emika panda arm
#
# Side view:
#     | |f------e--
#     | |--------\  \
#      g         | d|
#   -------      /  /
#   -------     / c/
#      h       /  /
#             /  /
#            | b|
#            |  |
#             a
#        ----------
#        -- base --
# data taken from:
# * https://www.franka.de/Panda_Datasheet_May_2019.pdf
# * https://frankaemika.github.io/docs/control_parameters.html to get an idea
# * https://github.com/frankaemika/franka_ros/tree/kinetic-devel/franka_description/robots



class FrankaEmikaPanda(Process):

    def __init__(self, name, parent, index = 0):
        # frame is the center of the base on the ground
        super().__init__(name, parent, index)
        # footrpint parameters
        # the arm is mounted at the center of the base
        self.base_x = 0.60
        self.base_y = 0.70
        self.base_z = 0.07
        # for the rest of the robot we use 3 cylinder and a cube for the last part
        # see https://frankaemika.github.io/docs/control_parameters.html to get an idea
        # footprint of the a+b part modeled as a cylinder
        self.ab_r = 0.13
        self.ab_h = 0.40
        # footprint of the c+d part modeled as a cylinder
        self.cd_r = 0.17
        self.cd_h = 0.40
        # footprint of the e+f part modeled as a cylinder
        self.ef_r = 0.17
        self.ef_h = 0.40
        # footprint of the g part as a cube
        self.g_x = 0.15
        self.g_y = 0.10
        self.g_z = 0.20
        # XXX we don't model the gripper
        # home offset (where the 0 position actually is compared to the neutral joint position)
        self.a_ref = 0.000000
        self.b_ref = -0.785398
        self.c_ref = -0.000000
        self.d_ref = -2.356195
        self.e_ref = 0.000000
        self.f_ref = 1.570796
        self.g_ref = 0.785398
        # state variables (see ascii art above)
        self._a = symbols(name + '_a')
        self._b = symbols(name + '_b')
        self._c = symbols(name + '_c')
        self._d = symbols(name + '_d')
        self._e = symbols(name + '_e')
        self._f = symbols(name + '_f')
        self._g = symbols(name + '_g')
        # angles limits
        self.minAngleA = -2.8973 - self.a_ref
        self.maxAngleA =  2.8973 - self.a_ref 
        self.minAngleB = -1.7628 - self.b_ref
        self.maxAngleB =  1.7628 - self.b_ref
        self.minAngleC = -2.8973 - self.c_ref
        self.maxAngleC =  2.8973 - self.c_ref
        self.minAngleD = -3.0718 - self.d_ref
        self.maxAngleD = -0.0698 - self.d_ref
        self.minAngleE = -2.8973 - self.e_ref
        self.maxAngleE =  2.8973 - self.e_ref
        self.minAngleF = -0.0175 - self.f_ref
        self.maxAngleF =  3.7525 - self.f_ref
        self.minAngleG = -2.8973 - self.g_ref
        self.maxAngleG =  2.8973 - self.g_ref
        # frame and stuff
        self._frame = parent.mountingPoint(index)
        self._af = self._frame.orient_new_axis( name + '_af', self._a + self.a_ref, self._frame.k, location= self.base_z * self._frame.k)
        self._bf = self._af.orient_new_axis(    name + '_bf', self._b + self.b_ref, self._af.j,    location= 0.333 * self._af.k)
        self._cf = self._bf.orient_new_axis(    name + '_cf', self._c + self.c_ref, self._bf.k,    location= 0.316 * self._bf.k)
        self._df = self._cf.orient_new_axis(    name + '_df', self._d + self.d_ref, self._cf.j,    location= 0.0825 * self._cf.i)
        self._ef = self._df.orient_new_axis(    name + '_ef', self._e + self.e_ref, self._df.k,    location=-0.0825 * self._df.i)
        self._ff = self._ef.orient_new_axis(    name + '_ff', self._f + self.f_ref, self._ef.j,    location= 0.384 * self._ef.k)
        self._effector = self._ff.locate_new(name + '_effector', 0.088 * self._ff.i)
        # motion primitives
        idle(self)
        wait(self)
        homePos(self)
        setJoints(self)
        closeGripper(self)
        openGripper(self)
    
    def frame(self):
        return self._frame
    
    def internalVariables(self):
        return [self._a, self._b, self._c, self._d, self._e, self._f, self._g]

    # min and max for all the angles
    def invariantG(self):
        domain_a = And(self._a >= self.minAngleA, self._a <= self.maxAngleA)
        domain_b = And(self._b >= self.minAngleB, self._b <= self.maxAngleB)
        domain_c = And(self._c >= self.minAngleC, self._c <= self.maxAngleC)
        domain_d = And(self._d >= self.minAngleD, self._d <= self.maxAngleD)
        domain_e = And(self._e >= self.minAngleE, self._e <= self.maxAngleE)
        domain_f = And(self._f >= self.minAngleF, self._f <= self.maxAngleF)
        domain_g = And(self._g >= self.minAngleG, self._g <= self.maxAngleG)
        return And(domain_a, domain_b, domain_c, domain_d, domain_e, domain_f, domain_g)
        
    def ownResources(self, point, delta = 0.0):
        lowerBackLeft1   = self._frame.origin.locate_new('franka_base_lbl',-self.base_x * self._frame.i / 2 - self.base_y * self._frame.j / 2 )
        upperFrontRight1 = self._frame.origin.locate_new('franka_base_ufr', self.base_x * self._frame.i / 2 + self.base_y * self._frame.j / 2 + self.base_z * self._frame.k )
        baseFP = cube(self._frame, lowerBackLeft1, upperFrontRight1, point, delta)
        #
        abFP = cylinder(self._af, self.ab_r, self.ab_h, point, delta)
        cdFP = cylinder(self._cf, self.cd_r, self.cd_h, point, delta)
        efFP = cylinder(self._ef, self.ef_r, self.ef_h, point, delta)
        #
        lowerBackLeft2   = self._effector.origin.locate_new('franka_effector_lbl',-self.base_x * self._effector.i / 2 - self.base_y * self._effector.j / 2 - self.base_z / 2 * self._effector.k )
        upperFrontRight2 = self._effector.origin.locate_new('franka_effector_ufr', self.base_x * self._effector.i / 2 + self.base_y * self._effector.j / 2 + self.base_z / 2 * self._effector.k )
        effectorFP = cube(self._effector, lowerBackLeft2, upperFrontRight2, point, delta)
        return Or(baseFP, abFP, cdFP, efFP, effectorFP)
    
    # overapproax of the workspace in https://www.franka.de/Panda_Datasheet_May_2019.pdf
    def abstractResources(self, point, delta = 0.0):
        # delta makes it bigger
        bottom = self._frame.locate_new('franka_bottom_workspace', -0.4 * self._frame.k)
        return cylinder(bottom, 0.9, 1.66, point, delta)
    
    def mountingPoint(self, index):
        assert(index == 0)
        return self._effector


class FrankaMotionPrimitive(MotionPrimitive):
    
    def __init__(self, name, component):
        super().__init__(name, component)
        self.err = 0.005

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


class homePos(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def setParameters(self, args):
        assert(len(args) in {0, 7})
        if len(args) == 0:
            return FrankaHomePos(self.name(), self._component)
        elif len(args) == 7:
            return FrankaMoveTo( self.name(), self._component,
                                 args[0], args[1], args[2], args[3], args[4], args[5], args[6],
                                 0, 0, 0, 0, 0, 0, 0)

class FrankaHomePos(FrankaMotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)
    
    def duration(self):
        return DurationSpec(0, 2, False) #TODO upper as function of the angle and speed

    def preG(self):
        return S.true

    def postG(self):
        return And(Eq(self._component._a, 0), Eq(self._component._b, 0), Eq(self._component._c, 0), Eq(self._component._d, 0), Eq(self._component._e, 0), Eq(self._component._f, 0), Eq(self._component._g, 0))



class idle(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def setParameters(self, args):
        assert(len(args) == 0)
        return FrankaIdle(self.name(), self._component)

class FrankaIdle(FrankaMotionPrimitive):

    def __init__(self, name, component):
        super().__init__(name, component)

    def modifies(self):
        return []

    def duration(self):
        return DurationSpec(0, float('inf'), True) 

    def preG(self):
        return S.true

    def postG(self):
        return S.true

    def invG(self):
        return S.true


# since we don't precisely model the gripper, it is like waiting
class closeGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["grasp width"]

    def setParameters(self, args):
        assert(len(args) == 1)
        return FrankaWait(self.name(), self._component)

# since we don't precisely model the gripper, it is like waiting
class openGripper(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def setParameters(self, args):
        assert(len(args) == 0)
        return FrankaWait(self.name(), self._component)

class wait(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return []

    def setParameters(self, args):
        assert(len(args) == 1)
        return FrankaWait(self.name(), self._component, args[0])

class FrankaWait(FrankaMotionPrimitive):

    def __init__(self, name, component, duration = None):
        super().__init__(name, component)
        if duration != None:
            self.duration = duration
        else:
            self.duration = 1

    def modifies(self):
        return []
    
    def duration(self):
        return DurationSpec(self.duration, self.duration, False)

    def preG(self):
        return S.true

    def postG(self):
        return S.true

    def invG(self):
        return S.true


class setJoints(MotionPrimitiveFactory):

    def __init__(self, component):
        super().__init__(component)

    def parameters(self):
        return ["a src", "b src", "c src", "d src", "e src", "f src", "g src",
                "a dst", "b dst", "c dst", "d dst", "e dst", "f dst", "g dst"]

    def setParameters(self, args):
        assert(len(args) == 14)
        return FrankaMoveTo( self.name(), self._component,
                             args[0], args[1], args[2], args[3], args[4], args[5], args[6],
                             args[7], args[8], args[9], args[10], args[11], args[12], args[13])

class FrankaMoveTo(FrankaMotionPrimitive):

    def __init__(self, name, component,
                 a0, b0, c0, d0, e0, f0, g0,
                 a1, b1, c1, d1, e1, f1, g1,
                 smooth = True):
        super().__init__(name, component)
        self.a0 = a0
        self.b0 = b0
        self.c0 = c0
        self.d0 = d0
        self.e0 = e0
        self.f0 = f0
        self.g0 = g0
        self.a1 = a1
        self.b1 = b1
        self.c1 = c1
        self.d1 = d1
        self.e1 = e1
        self.f1 = f1
        self.g1 = g1
        self.smooth = smooth
    
    def duration(self):
        return DurationSpec(0, 2, False) #TODO upper as function of the angle and speed

    def preG(self):
        return And(self.a0 - self.err <= self._component._a, self._component._a <= self.a0 + self.err,
                   self.b0 - self.err <= self._component._b, self._component._b <= self.b0 + self.err,
                   self.c0 - self.err <= self._component._c, self._component._c <= self.c0 + self.err,
                   self.d0 - self.err <= self._component._d, self._component._d <= self.d0 + self.err,
                   self.e0 - self.err <= self._component._e, self._component._e <= self.e0 + self.err,
                   self.f0 - self.err <= self._component._f, self._component._f <= self.f0 + self.err,
                   self.g0 - self.err <= self._component._g, self._component._g <= self.g0 + self.err)

    def invG(self, err = 0.1):
        cstr = S.true
        if self.smooth:
            t = timeSymbol()
            dt = self.duration().max
            ta = Eq(self._component._a, utils.transition.smoothstep(t, self.a0, self.a1, dt))
            tb = Eq(self._component._b, utils.transition.smoothstep(t, self.b0, self.b1, dt))
            tc = Eq(self._component._c, utils.transition.smoothstep(t, self.c0, self.c1, dt))
            td = Eq(self._component._d, utils.transition.smoothstep(t, self.d0, self.d1, dt))
            te = Eq(self._component._e, utils.transition.smoothstep(t, self.e0, self.e1, dt))
            tf = Eq(self._component._f, utils.transition.smoothstep(t, self.f0, self.f1, dt))
            tg = Eq(self._component._g, utils.transition.smoothstep(t, self.g0, self.g1, dt))
            cstr = And(t >= 0, t <= dt, ta, tb, tc, td, te, tf, tg)
        else:
            cstr = And(cstr, Min(self.a0, self.a1) - err <= self._component._a )
            cstr = And(cstr, self._component._a <= Max(self.a0, self.a1) + err )
            cstr = And(cstr, Min(self.b0, self.b1) - err <= self._component._b )
            cstr = And(cstr, self._component._b <= Max(self.b0, self.b1) + err )
            cstr = And(cstr, Min(self.c0, self.c1) - err <= self._component._c )
            cstr = And(cstr, self._component._c <= Max(self.c0, self.c1) + err )
            cstr = And(cstr, Min(self.d0, self.d1) - err <= self._component._d )
            cstr = And(cstr, self._component._d <= Max(self.d0, self.d1) + err )
            cstr = And(cstr, Min(self.e0, self.e1) - err <= self._component._e )
            cstr = And(cstr, self._component._e <= Max(self.e0, self.e1) + err )
            cstr = And(cstr, Min(self.f0, self.f1) - err <= self._component._f )
            cstr = And(cstr, self._component._f <= Max(self.f0, self.f1) + err )
            cstr = And(cstr, Min(self.g0, self.g1) - err <= self._component._g )
            cstr = And(cstr, self._component._g <= Max(self.g0, self.g1) + err )
            changing = []
            if self.a0 != self.a1:
                changing.append( (self.a0,self.a1,self._component._a) )
            if self.b0 != self.b1:
                changing.append( (self.b0,self.b1,self._component._b) )
            if self.c0 != self.c1:
                changing.append( (self.c0,self.c1,self._component._c) )
            if self.d0 != self.d1:
                changing.append( (self.d0,self.d1,self._component._d) )
            if self.e0 != self.e1:
                changing.append( (self.e0,self.e1,self._component._e) )
            if self.f0 != self.f1:
                changing.append( (self.f0,self.f1,self._component._f) )
            if self.g0 != self.g1:
                changing.append( (self.g0,self.g1,self._component._g) )
            for i in range(1, len(changing)):
                a0, a1, a = changing[0]
                b0, b1, b = changing[i] 
                cstr = And(cstr, Eq( (a - a0) / (a1 - a0), (b - b0) / (b1 - b0) ) )
        return cstr

    def postG(self, err = 0.0):
        return And(self.a1 - err <= self._component._a, self._component._a <= self.a1 + err,
                   self.b1 - err <= self._component._b, self._component._b <= self.b1 + err,
                   self.c1 - err <= self._component._c, self._component._c <= self.c1 + err,
                   self.d1 - err <= self._component._d, self._component._d <= self.d1 + err,
                   self.e1 - err <= self._component._e, self._component._e <= self.e1 + err,
                   self.f1 - err <= self._component._f, self._component._f <= self.f1 + err,
                   self.g1 - err <= self._component._g, self._component._g <= self.g1 + err)
