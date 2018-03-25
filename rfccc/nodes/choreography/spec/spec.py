from sympy import *
from sympy.vector import CoordSys3D
import functools
from abc import ABC, abstractmethod

######################################################################
# Some classes to write the spec of components and motion primitive. #
# This will be used for the compatibility check.                     #
######################################################################


class Component(ABC):
    
    def __init__(self, parent = None, index = 0):
        """abstract component
        parent -- the parent, if any (default None)
        index -- the index of the parent's mounting point (default 0)
        """
        self._parent = parent
        self._children = {}
        self._motionPrimitives = {}
        if parent != None:
            parent.addChildren(index, self)
    
    # returns a CoordSys3D
    @abstractmethod
    def frame(self):
        pass
    
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
        
    @abstractmethod
    def mountingPoint(self, index):
        """returns a coordinate system"""
        pass
    
    def addChildren(self, index, component):
        assert(not (index in self._children))
        self._children[index] = component
    
    def addMotionPrimitive(self, mp):
        n = mp.name()
        assert(self._motionPrimitives[n] == None)
        self._motionPrimitives[n] = mp
    
    def motionPrimitive(self, name):
        return self._motionPrimitives[name]


class MotionPrimitive():
    
    def __init__(self, component):
        component.addMP(self.__class__.__name__, self)
    
    def time():
        return Symbol("t")
    
    def name(self):
        return self.__class__.__name__
    
    def pre(self):
        """precondition over the component's variables"""
        return false
    
    def post(self):
        """postcondition over the component's variables"""
        return true
    
    def inv(self):
        """an invariant over the component's variables (as function of time)"""
        return true
    
    def preFP(self, point):
        """footprint of the precondition"""
        return true
    
    def postFP(self, point):
        """footprint of the postcondition"""
        return true
    
    def invFP(self, point):
        """footprint of the invariant"""
        return true
