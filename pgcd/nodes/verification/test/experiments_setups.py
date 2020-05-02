from sympy import *
from sympy.vector import CoordSys3D
from spec.component import *
from spec.motion import *
from spec.time import *
from utils.geometry import *
from cart import *
from arm import *
from mpmath import mp

# world: a trivial componenent (with optional mounting points as args)
class World(Component):

    def __init__(self, *mnts):
        super().__init__('World', None)
        f = CoordSys3D('World')
        self._frame = f
        self._mountingPoints = [f.orient_new_axis('world_mount_' + str(i), t, f.k, location= x * f.i + y * f.j + z * f.k) for i, (x,y,z,t) in enumerate(mnts)]
        self._mount = f

    def frame(self):
        return self._frame

    def mountingPoint(self, index):
        if index <= len(self._mountingPoints) :
            return self._mountingPoints[index]
        else:
            return self._frame

class DummyProcess(Process):

    def __init__(self, name, parent, index):
        super().__init__(name, parent, index)
        Idle(self)

    def frame(self):
        return self._parent.frame()

    def outputVariables(self):
        return [Symbol(self.name() + "_x"), Symbol(self.name() + "_y"), Symbol(self.name() + "_z")]

    def abstractResources(self, *arg):
        return S.false

    def mountingPoint(self, index):
        return self.frame()


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


def mkDummyWorld(*cmpNames):
    w = World()
    for i, name in enumerate(cmpNames):
        c = DummyProcess(name, world, i)
    return w
