from sympy import *
from sympy.vector import CoordSys3D
from spec import *
from geometry import *
from cart import *
from arm import *
from mpmath import mp

# world: a trivial componenent (with optional mounting points as args)
class World(Component):

    def __init__(self, *mnts):
        super().__init__('World', None)
        f = CoordSys3D('World')
        self._frame = f
        self._mountingPoints = [f.orient_new_axis('world_mount', t, f.k, location= x * f.i + y * f.j + z * f.k) for (x,y,z,t) in mnts]
        self._mount = f

    def frame(self):
        return self._frame

    def mountingPoint(self, index):
        if index <= len(self._mountingPoints) :
            return self._mountingPoints[index]
        else:
            return self._frame

def cartAndArmWorld():
    w = World()
    c = Cart("C", w)
    a = Arm("A", c)
    return w

def armsHandoverWorld():
    w = World( (-0.28,0,0,0), (0.28,0,0,mp.pi) )
    a1 = Arm("A1", w, 0)
    a2 = Arm("A2", w, 1)
    return w

def binSortingWorld():
    w = World( (0,0,0,mp.pi/2), (0.3,0,0,-mp.pi/2) )
    a1 = Arm("A", w, 0)
    a2 = Arm("B", w, 1)
    return w

def ferryWorld():
    w = World( (-1,-0.5,0,mp.pi/2), (1,-0.5,0,mp.pi/2) )
    a1 = Arm("A1", w, 0)
    a2 = Arm("A2", w, 1)
    c = Cart("C", w, 2)
    return w



# creating a few thing to test if it works:
def testing():
    print("cart and arm")
    w = cartAndArmWorld()
    print(w)
    print(w.allProcesses())
    print("arms hand over")
    w = armsHandoverWorld()
    print(w)
    print(w.allProcesses())
    print("bin sorting")
    w = binSortingWorld()
    print(w)
    print(w.allProcesses())
    print("ferry")
    w = ferryWorld()
    print(w)
    print(w.allProcesses())
