from sympy import *
from sympy.vector import CoordSys3D
import functools
from abc import ABC, abstractmethod
from utils.geometry import cube

######################################################################
# Some classes to write the spec of components and motion primitive. #
# This will be used for the compatibility check.                     #
######################################################################
    
def timeSymbol():
    return Symbol("t")

def timifyVar(var):
    return Function(var.name)(timeSymbol())
    
def timifyFormula(self, var, pred):
    time = { v: timifyVar(v) for v in var }
    return pred.subs(time)
    
def deTimifyFormula(var, pred):
    detime = { timifyVar(v): v for v in var }
    return pred.subs(detime)

class Component(ABC):
    
    def __init__(self, name, parent = None, index = 0):
        """abstract component
        name -- an unique identifier
        parent -- the parent, if any (default None)
        index -- the index of the parent's mounting point (default 0)
        """
        self._name = name
        self._parent = parent
        self._children = {}
        if parent != None:
            parent.addChildren(index, self)
        self._obstacles = []

    def name(self):
        return self._name
    
    # returns a CoordSys3D
    @abstractmethod
    def frame(self):
        pass

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
        cp = [p for c in self._children.values() for p in c.allProcesses()]
        if self.isProcess():
           cp.append(self) 
        return cp

    def obstacles(self):
        cp = [p for c in self._children.values() for p in c.allProcesses()]
        if self.isObstacle():
           cp.append(self) 
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
        return []
    
    def outputVariables(self):
        """returns a list output variables as sympy symbols"""
        return []

    def ownVariables(self):
        """returns all the internal and output as sympy symbols"""
        return self.internalVariables() + self.outputVariables()
    
    def variables(self):
        """returns all the variables as sympy symbols"""
        return self.internalVariables() + self.inputVariables() + self.outputVariables()

    def invariant(self):
        """returns some constraints over the variables"""
        #TODO in the future we'll need to split this in assumption and guarantees depending on the variables
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

# Specification for the timing aspect of a motion primitive
class DurationSpec():

    def __init__(self, _min = 0, _max = float('inf'), interruptible = True):
        self.min = _min
        self.max = _max
        self.interruptible = interruptible

    def __str__(self):
        return "DurationSpec(" + str(self.min) + ", " + str(self.max) + ", " + str(self.interruptible) + ")"

    def copy(self):
        return DurationSpec(self.min, self.max, self.interruptible)

    def valid(self):
        return self.min <= self.max

    def concat(self, ds):
        assert not self.interruptible, "cannot add something after an non interruptible motion: " + str(self) + " then " + str(ds)
        if ds.interruptible:
            new_min = self.max + ds.min
            new_max = self.min + ds.max
            assert(new_min <= new_max)
            return DurationSpec(new_min, new_max, ds.interruptible)
        else:
            return DurationSpec(self.min + ds.min, self.max + ds.max, ds.interruptible)

    def intersect(self, ds):
        assert(self.interruptible or ds.interruptible)
        if not self.interruptible:
            assert(ds.min <= self.min and ds.max >= self.max), "(1) " + str(self) + " ∩ " + str(ds)
            return DurationSpec(self.min, self.max, False)
        elif not ds.interruptible:
            assert(self.min <= ds.min and self.max >= ds.max), "(2) " + str(self) + " ∩ " + str(ds)
            return DurationSpec(ds.min, ds.max, False)
        else:
            new_min = max(self.min, ds.min)
            new_max = min(self.max, ds.max)
            assert(new_min <= new_max), "(3) " + str(self) + " ∩ " + str(ds)
            return DurationSpec(new_min, new_max, True)

    def implements(self, ds):
        same = self.interruptible == ds.interruptible
        if self.interruptible:
            return same and self.min <= ds.min and self.max >= ds.max
        else:
            return same and self.min >= ds.min and self.max <= ds.max

# Some motion primitives have parameters, we represent that with a factory.
# Given some concrete value for the parameters we get a motion primitive.
# Currently the values should be concrete, formula not yet supported
class MotionPrimitiveFactory(ABC):

    def __init__(self, component):
        self._component = component
        component.addMotionPrimitive(self)
    
    def name(self):
        return self.__class__.__name__

    def parameters(self):
        return []

    # returns a MotionPrimitive
    @abstractmethod
    def setParameters(self, args):
        pass

class MotionPrimitive():
    
    def __init__(self, name, component):
        self._name = name
        self._component = component
    
    def name(self):
        return self._name

    def timify(self, pred):
        time = { var: timifyVar(var) for var in self._component.variables() }
        return pred.subs(time)

    def duration(self):
        return DurationSpec(1, 1, False)

    def modifies(self):
        '''some motion primitive (like idle) does not change all the variables'''
        return self._component.ownVariables()
    
    def pre(self):
        """precondition over the component's variables"""
        return S.false
    
    def post(self):
        """postcondition over the component's variables"""
        return S.true
    
    def inv(self):
        """an invariant over the component's variables (as function of time)"""
        return S.true
    
    def preFP(self, point):
        """footprint of the precondition"""
        return S.true
    
    def postFP(self, point):
        """footprint of the postcondition"""
        return S.true
    
    def invFP(self, point):
        """footprint of the invariant"""
        return S.true

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
        return cube(cubeFrame, cf.origin, cf.origin.locate_new(self.dx * cf.i + self.dy * cf.j + self.dz * cf.k) , point)
