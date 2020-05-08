from sympy import *
from abc import ABC, abstractmethod
from spec.time import timifyVar
from spec.contract import *
import spec.conf

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

class MotionPrimitive(AssumeGuaranteeContract):
    
    def __init__(self, name, component):
        super().__init__(name)
        self._component = component
    
    def name(self):
        return self._name

    def components(self):
        return {self._component}

    def timify(self, pred):
        time = { var: timifyVar(var) for var in self._component.variables() }
        return pred.subs(time)

    def modifies(self):
        '''some motion primitive (like idle) does not change all the variables'''
        return self._component.ownVariables()
    
    def wellFormed(self, extra = ExtraInfo()):
        vcs = super().wellFormed(extra)
        if spec.conf.enableMPincludeFPCheck:
            # checks that the components FP is in the motion primitive MP
            prefix = self.name + " well-formed: contains component FP "
            px, py, pz = symbols('inFpX inFpY inFpZ')
            frame = self.frame()
            point = frame.origin.locate_new("inFp", px * frame.i + py * frame.j + pz * frame.k )
            pointDomain = And(px >= minX, px <= maxX, 
                              py >= minY, py <= maxY,
                              pz >= minZ, pz <= maxZ)
            #pre
            pre = And(pointDomain,
                      self._component.invariantG(),
                      self.preG(),
                      Not(self.preFP(point)),
                      extra.pre,
                      extra.always)
            vcs.append( VC(prefix + "pre", [And(pre, self._component.abstractResources(point)),
                                            And(pre, self._component.ownResources(point))]) )
            #inv
            # no quantification over time for the moment
            assert(self.isInvTimeInvariant())
            inv = And(pointDomain,
                      self._component.invariantG(),
                      self.deTimifyFormula(self.invG()),
                      Not(self.deTimifyFormula(self.invFP(point))),
                      self.deTimifyFormula(extra.inv),
                      extra.always)
            vcs.append( VC(prefix + "inv",  [And(inv, self._component.abstractResources(point)),
                                             And(inv, self._component.ownResources(point))]) )
            # post
            post = And(pointDomain,
                       self._component.invariantG(),
                       self.postG(),
                       Not(self.postFP(point)),
                       extra.post,
                       extra.always)
            vcs.append( VC(prefix + "post", [And(post, self._component.abstractResources(point)),
                                             And(post, self._component.ownResources(point))]) )
        return vcs
