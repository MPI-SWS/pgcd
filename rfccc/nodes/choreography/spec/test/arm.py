from sympy import *
from sympy.vector import CoordSys3D
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
        self.lowerArmLength = 0.2
        self.lowerArmRadius = 0.05
        # variables
        self._a = symbols(name + '_a') # rotation along the Y-axis (upper arm to lower arm)
        self._b = symbols(name + '_b') # rotation along the Y-axis (base to upper arm)
        self._c = symbols(name + '_c') # rotation along the Z-axis (base)
        # frame and stuff
        self._frame = parent.mountingPoint(index)
        self._base = self._frame.orient_new_axis(name + '_base', self._c, self._frame.k)
        self._upper = self._base.orient_new_axis(name + '_upper', self._b, self._base.j, location= self.baseHeight * self._base.k)
        self._lower = self._upper.orient_new_axis(name + '_lower', self._a, self._upper.j, location= self.upperArmLength * self._upper.k)
        self._effector = self._lower.locate_new(name + '_effector', self.lowerArmLength * self._lower.k)
    
    def frame(self):
        return self._frame
    
    def ownResources(self, point):
        baseFP = cylinder(self._base, self.baseRadius, self.baseHeight, point)
        upperFP = cylinder(self._upper, self.upperArmRadius, self.upperArmLength, point)
        lowerFP = cylinder(self._lower, self.lowerArmRadius, self.lowerArmLength, point)
        return Or(baseFP, upperFP, lowerFP)
    
    def abstractResources(self, point):
        f = self.frame()
        r = self.baseHeight + self.upperArmLength + self.lowerArmLength #roughly
        return halfSphere(f, r, f.k, point)
    
    def mountingPoint(self, index):
        assert(index == 0)
        return self._effector
