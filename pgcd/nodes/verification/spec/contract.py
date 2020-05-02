from sympy import *
from abc import ABC, abstractmethod
from spec.time import *
from utils.vc import *

# default bounds for the footprint
# TODO make modifiable
minX = -10
maxX =  10
minY = -10
maxY =  10
minZ = 0
maxZ = 2

class AssumeGuaranteeContract(ABC):

    # Part about the processes for this contract

    def __init__(self, name):
        self.name = name

    @abstractmethod
    def components(self):
        """The component part of this contract"""
        pass

    def frame(self):
        """The frame in which this contract is expressed. By default take the frame from one of the components."""
        return next(iter(self.components())).frame()
# TODO impact of the frame ?

    def inputs(self):
        """external inputs"""
        return { v for c in self.components() for v in c.inputVariables() }
    
    def outputs(self):
        """outputs"""
        return { v for c in self.components() for v in c.outputVariables() }

    def allVariables(self):
        return self.inputs() | self.outputs()

    # Part about the pre/post/inv

    def duration(self):
        return DurationSpec(1, 1, False)

    def preA(self):
        """assumption over the contract inputs at the beginning"""
        return S.true

    def preG(self):
        """guarantees over the contract outputs at the beginning"""
        return S.false # without overriding preG, the contract is empty!

    def preFP(self, point):
        """footprint at the start"""
        return S.true

    def invA(self):
        """assumption during the execution of the contract (as function of time)"""
        return S.true

    def invG(self):
        """guarantees during the execution of the contract (as function of time)"""
        return S.true

    def invFP(self, point):
        """footprint of the invariant"""
        return S.true
    
    def isInvTimeInvariant(self):
        """Are the invariant constraints fime invariant"""
        #TODO for the moment, what we have is ok but it needs to be checked
        return True

    def deTimifyFormula(self, formula):
        return deTimifyFormula(self.allVariables(), formula)

    def postA(self):
        """assumption when the contract terminates"""
        return S.true

    def postG(self):
        """guarantees when the contract terminates"""
        return S.true

    def postFP(self, point):
        """footprint of the postcondition"""
        return S.true

    def wellFormed(self):
        """check that the contract is not empty, returns VCs """
        # title
        prefix = self.name + " well-formed: "
        # no quantification over time or quantifier alternation for the moment
        assert(self.isInvTimeInvariant())
        assert(self.preA().free_symbols.isdisjoint(self.preG().free_symbols))
        assert(self.deTimifyFormula(self.invA()).free_symbols.isdisjoint(self.deTimifyFormula(self.invG()).free_symbols))
        assert(self.postA().free_symbols.isdisjoint(self.postG().free_symbols))
        vcs = [
            VC(prefix + "components", [sympify(len(self.components()) > 0)], True),
            VC(prefix + "duration", [sympify(self.duration().valid())], True),
            #pre
            VC(prefix + "preA",   [self.preA()], True),
            VC(prefix + "preG",   [self.preG()], True),
            # inv
            VC(prefix + "invA",   [self.deTimifyFormula(self.invA())], True),
            VC(prefix + "invG",   [self.deTimifyFormula(self.invG())], True),
            # post
            VC(prefix + "postA",  [self.postA()], True),
            VC(prefix + "postG",  [self.postG()], True)
        ]
        return vcs

    def refines(self, contract):
        """check refinement, returns VCs"""
        # title
        prefix = self.name + " refines " + contract.name + ": "
        # for the footprint
        px, py, pz = symbols('inFpX inFpY inFpZ')
        frame = self.frame() #TODO equality of the two frames ?
        point = frame.origin.locate_new("inFp", px * frame.i + py * frame.j + pz * frame.k )
        pointDomain = And(px >= minX, px <= maxX, 
                          py >= minY, py <= maxY,
                          pz >= minZ, pz <= maxZ)
        # no quantification over time for the moment
        assert(self.isInvTimeInvariant())
        assert(contract.isInvTimeInvariant())
        # TODO something feels wrong here: ...
        vcs = [
            VC(prefix + "components", [sympify(self.components() == contract.components())], True),
            VC(prefix + "duration", [sympify(self.duration().implements(contract.duration()))], True),
            #pre
            VC(prefix + "preA",   [And(contract.preA(), Not(self.preA()))]), # contract.A ⇒ self.A
            VC(prefix + "preG",   [And(self.preG(), Not(contract.preG()))]), # self.G ⇒ contract.G
            VC(prefix + "preFP",  [And(pointDomain, self.preFP(point), Not(contract.preFP(point)))]), # self.FP ⊆ contract.FP, TODO strengthen by preG
            # inv
            VC(prefix + "invA",   [And(contract.deTimifyFormula(contract.invA()), Not(self.deTimifyFormula(self.invA())))]),
            VC(prefix + "invG",   [And(self.deTimifyFormula(self.invG()), Not(contract.deTimifyFormula(contract.invG())))]),
            VC(prefix + "invFP",  [And(pointDomain, self.deTimifyFormula(self.invFP(point)), Not(contract.deTimifyFormula(contract.invFP(point))))]), #TODO strengthen by invG
            # post
            VC(prefix + "postA",  [And(contract.postA(), Not(self.postA()))]),
            VC(prefix + "postG",  [And(self.postG(), Not(contract.postG()))]),
            VC(prefix + "postFP", [And(pointDomain, self.postFP(point), Not(contract.postFP(point)))]) # TODO strengthen by postG
        ]
        return vcs




class ComposedContract(AssumeGuaranteeContract):
    """returns a new contract which is the composition of two contracts"""

    def __init__(self, contract1, contract2, connection):
        super().__init__("composition of " + contract1.name + " and " + contract2.name)
        self._contract1 = contract1
        self._contract2 = contract2
        self._connection = connection
        # sanity checks
        assert contract1.components().isdisjoint(contract2.components())
        self._contract1.duration().intersect(self._contract2.duration()) # duration intersect
        # TODO connection is valid
        # TODO compatible G ⇒ A
        # TODO not vacuous
        # TODO the frames are related ?
    
    def components(self):
        c1 = self._contract1.components()
        c2 = self._contract2.components()
        return c1 | c2

    def inputs(self):
        i1 = self._contract1.inputs() - self._contract2.outputs()
        i2 = self._contract2.inputs() - self._contract1.outputs()
        return i1 | i2
    
    def outputs(self):
        o1 = self._contract1.outputs()
        o2 = self._contract2.outputs()
        return o1 | o2

    def duration(self):
        d1 = self._contract1.duration()
        d2 = self._contract2.duration()
        return d1.intersect(d2)

    def preA(self):
        a1 = Implies(self._contract2.preG(), self._contract1.preA())
        a2 = Implies(self._contract1.preG(), self._contract2.preA())
        return And(a1, a2)

    def preG(self):
        g1 = self._contract1.preG()
        g2 = self._contract2.preG()
        return And(g1, g2)

    def preFP(self, point):
        fp1 = self._contract1.preFP(point)
        fp2 = self._contract2.preFP(point)
        return Or(fp1, fp2)

    def invA(self):
        a1 = Implies(self._contract2.invG(), self._contract1.invA())
        a2 = Implies(self._contract1.invG(), self._contract2.invA())
        return And(a1, a2)

    def invG(self):
        g1 = self._contract1.invG()
        g2 = self._contract2.invG()
        return And(g1, g2)

    def invFP(self, point):
        fp1 = self._contract1.invFP(point)
        fp2 = self._contract2.invFP(point)
        return Or(fp1, fp2)

    def postA(self):
        a1 = Implies(self._contract2.postG(), self._contract1.postA())
        a2 = Implies(self._contract1.postG(), self._contract2.postA())
        return And(a1, a2)

    def postG(self):
        g1 = self._contract1.postG()
        g2 = self._contract2.postG()
        return And(g1, g2)

    def postFP(self, point):
        fp1 = self._contract1.postFP(point)
        fp2 = self._contract2.postFP(point)
        return Or(fp1, fp2)

    def checkCompatibility(self):
        """returns VCs to check that two contracts are compatible"""
        pass #TODO
