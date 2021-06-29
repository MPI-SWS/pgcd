from sympy import *
from sympy.vector import CoordSys3D
import functools
from abc import ABC, abstractmethod
from verification.utils.geometry import cube
import verification.spec.conf


class Component(ABC):

    def __init__(self, name, parent = None, index = 0):
        """abstract component
        name -- an unique identifier
        parent -- the parent, if any (default None)
        index -- the index of the parent's mounting point (default 0)
        """
        self._name = name
        self._parent = parent
        self._index = index
        self._children = {}
        if parent != None:
            parent.addChildren(index, self)
        self._obstacles = []

    def name(self):
        return self._name

    # returns a CoordSys3D
    def frame(self):
        if self._parent != None:
            return self._parent.mountingPoint(self._index)
        else:
            return verification.spec.conf.worldFrame

    @abstractmethod
    def mountingPoint(self, index):
        """returns a coordinate system"""
        pass

    def addChildren(self, index, component):
        assert(not (index in self._children))
        self._children[index] = component

    def isProcess(self):
        return S.false

    def isObstacle(self):
        return S.false

    def allProcesses(self):
        cp = { p for c in self._children.values() for p in c.allProcesses() }
        if self.isProcess():
           cp.add(self)
        return cp

    def obstacles(self):
        cp = { p for c in self._children.values() for p in c.obstacles() }
        if self.isObstacle():
           cp.add(self)
        return cp


# Currently the convention is that processes should prefix their variable by "$name_".
# In the future, we may check/automate that part

class Process(Component):

    def __init__(self, name, parent = None, index = 0):
        super().__init__(name, parent, index)
        self._connections = {}
        self._motionPrimitives = {}

    def isProcess(self):
        return S.true

    def internalVariables(self):
        """returns a list internal variables as sympy symbols"""
        return []

    def inputVariables(self):
        """returns a list input variables as sympy symbols"""
        # TODO the frame
        return []

    def outputVariables(self):
        """returns a list output variables as sympy symbols"""
        # TODO the mounting points
        return []

    def ownVariables(self):
        """returns all the internal and output as sympy symbols"""
        return self.internalVariables() + self.outputVariables()

    def variables(self):
        """returns all the variables as sympy symbols"""
        return self.internalVariables() + self.inputVariables() + self.outputVariables()

    def invariantA(self):
        """returns some constraints over the input variables"""
        return S.true

    def invariantG(self):
        """returns some constraints over the output variables"""
        return S.true

    def ownResources(self, point):
        """returns constraints that are true if point is in the resources of this process"""
        return S.true

    def allResources(self, point):
        """returns constraints that are true if point is in the resources of this process or its children"""
        own = self.ownResources(point)
        childrenRes = [x.allResources(point) for x in self._children.values()]
        return functools.reduce(Or, childrenRes, own)

    def abstractResources(self, point):
        """an overapproximation of the resources, easier to solve"""
        return self.ownResources(point)

    def connect(self, index, component, connection = {}):
        self.addChildren(index, component)
        self._connections[index] = connection

    def addMotionPrimitive(self, mp):
        n = mp.name()
        assert(not (n in self._motionPrimitives))
        self._motionPrimitives[n] = mp

    def motionPrimitive(self, name, *args):
        return self._motionPrimitives[name].setParameters(args)

    def motionPrimitiveWithFreeParams(self, name):
        factory = self._motionPrimitives[name]
        params = []
        base = self.name() + "_" + name + "_"
        counter = 0
        for p in factory.parameters():
            ptName = base + str(counter)
            px = Symbol(ptName + "_x")
            py = Symbol(ptName + "_y")
            pz = Symbol(ptName + "_z")
            f = self.frame()
            pt = f.origin.locate_new(ptName, px * f.i + py * f.j + px * f.k)
            params.append(pt)
            counter += 1
        return factory.setParameters(params), params


# Passive object in the environement which have a footprint (to check collision)
class Obstacle(Component):

    def __init__(self, name, parent = None, index = 0):
        super().__init__(name, parent, index)

    def isObstacle(self):
        return True

    def frame(self):
        return self._parent.frame() #by default use the parent's frame

    def footprint(self, point):
        """footprint of the object"""
        return S.true

    def mountingPoint(self, index):
        return ValueException(self.name() + " does not have mounting moints.")


class Cube(Obstacle):

    count = 0

    def __init__(self, x, y, z, theta, dx, dy, dz, parent = None, index = 0):
        super().__init__("cube_" + str(Cube.count), parent, index)
        self.count = Cube.count
        Cube.count += 1
        self.x = x
        self.y = y
        self.z = z
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.theta = theta

    def footprint(self, point):
        f = self.frame()
        cf = f.orient_new_axis(self.name(), self.theta, f.k, location = self.x * f.i + self.y * f.j + self.z * f.k)
        return cube(cf, cf.origin, cf.origin.locate_new(self.name() + "_top", self.dx * cf.i + self.dy * cf.j + self.dz * cf.k) , point)

# world: a trivial componenent (with optional mounting points as args)
class World(Component):

    def __init__(self, *mnts):
        super().__init__('World', None)
        f = verification.spec.conf.worldFrame
        self._frame = f
        self._mountingPoints = [f.orient_new_axis('world_mount_' + str(i), t, f.k, location= x * f.i + y * f.j + z * f.k) for i, (x,y,z,t) in enumerate(mnts)]
        self._mount = f

    def frame(self):
        return self._frame

    def mountingPoint(self, index):
        if index < len(self._mountingPoints) :
            return self._mountingPoints[index]
        else:
            return self._frame

