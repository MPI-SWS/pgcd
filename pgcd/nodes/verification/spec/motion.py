from sympy import *
from abc import ABC, abstractmethod
from spec.time import timifyVar

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
