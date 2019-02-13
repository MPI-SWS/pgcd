from sympy import *
from sympy.vector import CoordSys3D
from spec import *
from utils.geometry import *
from cart import *
from arm import *

# world: a trivial componenent
class World(Component):

    def __init__(self):
        super().__init__('World', None)
        self._frame = CoordSys3D('World')

    def frame(self):
        return self._frame

    def mountingPoint(self, index):
        return self._frame


# creating a few thing to test if it works:
def main():
    print("Creating world")
    w = World()
    print("Creating cart")
    c = Cart("C", w)
    print("Creating arm")
    a = Arm("A", c)
    print("Processes in the world")
    print(w.allProcesses())
    print("arm footprint")
    px, py, pz = symbols('px py pz')
    pt = w.frame().origin.locate_new("p", px * w.frame().i + py * w.frame().j + pz * w.frame().k)
    #trigsimp(a.ownResources(pt))
    #simplify(a.ownResources(pt))
    print(a.ownResources(pt))
    print("arm abstract footprint")
    print(a.abstractResources(pt))
    print("arm invariant")
    print(a.invariant())
    print("cart footprint")
    print(c.ownResources(pt))
    print("cart + children footprint")
    print(c.allResources(pt))
    loc = w.frame().origin.locate_new("loc", w.frame().i * 5 )
    mp = c.motionPrimitive("MoveFromTo", w.frame().origin, loc)
    print("cart -> move -> pre")
    print(mp.pre())
    print("cart -> move -> preFP")
    print(mp.preFP(pt))
    print("cart -> move -> post")
    print(mp.post())
    print("cart -> move -> postFP")
    print(mp.postFP(pt))
    print("cart -> move -> inv")
    print(mp.inv())
    print("cart -> move -> invFP")
    print(mp.invFP(pt))

main()
