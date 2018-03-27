from sympy import *
from sympy.vector import CoordSys3D
import functools
from abc import ABC, abstractmethod

######################################################################
# Some classes to write the spec of components and motion primitive. #
# This will be used for the compatibility check.                     #
######################################################################

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
        self._motionPrimitives = {}
        if parent != None:
            parent.addChildren(index, self)

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
        return False

    def allProcesses(self):
        cp = [p for c in self._children.values() for p in c.allProcesses()]
        if self.isProcess():
           cp.append(self) 
        return cp


class Process(Component):
    
    def __init__(self, name, parent = None, index = 0):
        super().__init__(name, parent, index)
    
    def isProcess(self):
        return True

    def internalVariables(self):
        """returns a list internal variables as sympy symbols"""
        return []
    
    def inputVariables(self):
        """returns a list input variables as sympy symbols"""
        return []
    
    def outputVariables(self):
        """returns a list output variables as sympy symbols"""
        return []
    
    def variables(self):
        """returns all the variables as sympy symbols"""
        return internalVariables + inputVariables + outputVariables
    
    def ownResources(self, point):
        """returns constraints that are true if point is in the resources of this process"""
        return true
    
    def allResources(self, point):
        """returns constraints that are true if point is in the resources of this process or its children"""
        own = self.ownResources(point)
        childrenRes = [x.allResources(point) for x in self._children.values()]
        return functools.reduce(Or, childrenRes, own)
    
    def abstractResources(self, point):
        """an overapproximation of the resources, easier to solve"""
        return true
    
    def addMotionPrimitive(self, mp):
        n = mp.name()
        assert(not (n in self._motionPrimitives))
        self._motionPrimitives[n] = mp
    
    def motionPrimitive(self, name):
        return self._motionPrimitives[name]


class MotionPrimitiveFactory(ABC):

    def __init__(self, component):
        self._component = component
        component.addMotionPrimitive(self)
    
    def name(self):
        return self.__class__.__name__

    def parameters(self):
        return []

    # returns a MotionPrimitive
    def setParameters(self, *args):
        pass

class MotionPrimitive():
    
    def __init__(self, component):
        self._component = component
    
    def timeSymbol(self):
        return Symbol("t")

    def timify(self, var):
        return Function(var.name)(self.timeSymbol())

    def duration(self):
        return Int(1)
    
    def pre(self):
        """precondition over the component's variables"""
        return False
    
    def post(self):
        """postcondition over the component's variables"""
        return True
    
    def inv(self):
        """an invariant over the component's variables (as function of time)"""
        return True
    
    def preFP(self, point):
        """footprint of the precondition"""
        return True
    
    def postFP(self, point):
        """footprint of the postcondition"""
        return True
    
    def invFP(self, point):
        """footprint of the invariant"""
        return True
