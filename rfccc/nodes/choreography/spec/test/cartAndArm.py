from sympy import *
from sympy.vector import CoordSys3D
from spec import *
from geometry import *

# world: a trivial componenent
class World(Component):

    def __init__(self):
        super().__init__(None)
        self._frame = CoordSys3D('World')

    def frame(self):
        return self._frame

    def mountingPoint(self, index):
        return self._frame

# cart
class Cart(Component):
    
    def __init__(self, name, world, index = 0):
        super().__init__(world, index)
        self._name = name
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
    
    def frame(self):
        # reuse the parent's position
        return self._parent.frame()
    
    def outputVariables(self):
        return [self._x, self._y, self._theta]
    
    def mountingPoint(self, index):
        assert(index == 0)
        return self._mount
    
    def ownResources(self, point):
        return cylinder(self.frame(), self.radius, self.height, point)

# model for a generic arm as 3 linkage connected by revolute actuators
class Arm(Component):
    
    def __init__(self, name, parent, index = 0):
        super().__init__(parent, index)
        self._name = name
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


# creating a few thing to test if it works:
def main():
    print("Creating world")
    w = World()
    print("Creating cart")
    c = Cart("C", w)
    print("Creating arm")
    a = Arm("A", c)
    print("arm footprint")
    px, py, pz = symbols('px py pz')
    pt = w.frame().origin.locate_new("p", px * w.frame().i + py * w.frame().j + pz * w.frame().k)
    #trigsimp(a.ownResources(pt))
    #simplify(a.ownResources(pt))
    print(a.ownResources(pt))
    print("arm abstract footprint")
    print(a.abstractResources(pt))
    print("cart footprint")
    print(c.ownResources(pt))
    print("cart + children footprint")
    print(c.allResources(pt))

main()
