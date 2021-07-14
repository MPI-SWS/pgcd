from sympy import *
from abc import ABC, abstractmethod
from verification.spec.time import *
from verification.utils.vc import *
import verification.spec.conf as conf


#TODO in the long term we should get rid of this, and instead, update the contract with the extra info
class ExtraInfo():
    """Some extra info which can be used during well-formed and refinement check"""

    def __init__(self,
                 pre = S.true,
                 inv = S.true,
                 post = S.true,
                 always = S.true):
        self.pre = pre
        self.inv = inv
        self.post = post
        self.always = always

    def strengthen(self, extra):
        return ExtraInfo(And(self.pre, extra.pre),
                         And(self.inv, extra.inv),
                         And(self.post, extra.post),
                         And(self.always, extra.always))

    def __str__(self):
        return "ExtraInfo\n\tpre : " + str(self.pre) + "\n\tinv : " + str(self.inv) + "\n\tpost: " +  str(self.post) + "\n\tall : " + str(self.always)

def findLeastAncestorFrame(components):
    for p in components:
        children = p.allProcesses() #aslo contains p
        if components.issubset(children):
            return p.frame()
    return conf.worldFrame

class AssumeGuaranteeContract(ABC):

    # Part about the processes for this contract

    def __init__(self, name):
        self.name = name

    @abstractmethod
    def components(self):
        """The component part of this contract"""
        pass

    def frame(self):
        """The frame in which this contract is expressed. By default take the frame from one of the components.
           As default, take the least common ancestor of the components or the world frame if there is none."""
        return findLeastAncestorFrame(self.components())

    def inputs(self):
        """external inputs"""
        return { v for c in self.components() for v in c.inputVariables() }

    def outputs(self):
        """outputs"""
        return { v for c in self.components() for v in c.outputVariables() }

    def allVariables(self):
        return self.inputs() | self.outputs()

    def reallyAllVariables(self): # this includes the internal variables of the components
        return { v for c in self.components() for v in c.variables() }

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
        return deTimifyFormula(self.reallyAllVariables(), formula)

    def postA(self):
        """assumption when the contract terminates"""
        return S.true

    def postG(self):
        """guarantees when the contract terminates"""
        return S.true

    def postFP(self, point):
        """footprint of the postcondition"""
        return S.true

    def postErrA(self):
        """[optional] assumption when the contract fails"""
        return None

    def postErrG(self):
        """[optional] guarantees when the contract fails"""
        return None

    def postErrFP(self, point):
        """[optional] footprint of the error postcondition"""
        return None

    def wellFormed(self, extra = ExtraInfo()):
        """check that the contract is not empty, returns VCs """
        # title
        prefix = self.name + " well-formed: "
        # no quantification over time or quantifier alternation for the moment
        assert(self.isInvTimeInvariant())
        assert(self.preA().free_symbols.isdisjoint(self.preG().free_symbols))
        assert(self.deTimifyFormula(self.invA()).free_symbols.isdisjoint(self.deTimifyFormula(self.invG()).free_symbols))
        assert(self.postA().free_symbols.isdisjoint(self.postG().free_symbols))
        vcs = []
        # has at least one component
        vcs.append( VC(prefix + "components", [sympify(len(self.components()) > 0)], True) )
        # valid duration
        if conf.enableDurationCheck:
            vcs.append(VC(prefix + "duration", [sympify(self.duration().valid())], True))
        # pre
        vcs.append( VC(prefix + "preA",   [And(self.preA(), extra.pre, extra.always)], True) )
        vcs.append( VC(prefix + "preG",   [And(self.preG(), extra.pre, extra.always)], True) )
        # inv
        vcs.append( VC(prefix + "invA",   [And(self.deTimifyFormula(self.invA()), self.deTimifyFormula(extra.inv), extra.always)], True) )
        vcs.append( VC(prefix + "invG",   [And(self.deTimifyFormula(self.invG()), self.deTimifyFormula(extra.inv), extra.always)], True) )
        # post
        vcs.append( VC(prefix + "postA",  [And(self.postA(), extra.post, extra.always)], True) )
        vcs.append( VC(prefix + "postG",  [And(self.postG(), extra.post, extra.always)], True) )
        # post err (if exists)
        assert((self.postErrA() == None) == (self.postErrG() == None))
        assert((self.postErrA() == None) or (self.postErrA().free_symbols.isdisjoint(self.postErrG().free_symbols)))
        if self.postErrA() != None:
            vcs.append( VC(prefix + "postErrA",  [And(self.postErrA(), extra.always)], True) ) #extra.post_err ?
            vcs.append( VC(prefix + "postErrA ⇒ invA",  [And(self.deTimifyFormula(self.invA()), self.deTimifyFormula(extra.inv), Not(self.postErrA()), extra.always)], False) ) #extra.post_err ?
        if self.postErrG() != None:
            vcs.append( VC(prefix + "postErrG",  [And(self.postErrG(), extra.always)], True) ) #extra.post_err ?
            vcs.append( VC(prefix + "postErrG ⇒ invG",  [And(self.deTimifyFormula(self.invG()), self.deTimifyFormula(extra.inv), Not(self.postErrG()), extra.always)], False) ) #extra.post_err ?
        px, py, pz = symbols('inFpX inFpY inFpZ')
        frame = self.frame()
        point = frame.origin.locate_new("inFp", px * frame.i + py * frame.j + pz * frame.k )
        assert((self.postErrA() == None) == (self.postErrFP(point) == None))
        if self.postErrFP(point) != None:
            # for the footprint
            pointDomain = And(px >= conf.minX, px <= conf.maxX,
                              py >= conf.minY, py <= conf.maxY,
                              pz >= conf.minZ, pz <= conf.maxZ)
            inv = And(pointDomain,
                      self.deTimifyFormula(self.invA()),
                      self.deTimifyFormula(self.invG()),
                      self.deTimifyFormula(self.invFP(point)),
                      self.deTimifyFormula(extra.inv))
            post_err = And(self.postErrA(),
                           self.postErrG(),
                           Not(self.postErrFP(point))) #extra.post_err ?
            vcs.append( VC(prefix + "postErrFP ⊆ invFP",  [And(inv, post_err, extra.always)], False) )
        return vcs

    def refines(self, contract, extra = ExtraInfo()):
        """check refinement, returns VCs"""
        # title
        prefix = self.name + " refines " + contract.name + ": "
        # for the footprint
        px, py, pz = symbols('inFpX inFpY inFpZ')
        frame = self.frame()
        point = frame.origin.locate_new("inFp", px * frame.i + py * frame.j + pz * frame.k )
        pointDomain = And(px >= conf.minX, px <= conf.maxX,
                          py >= conf.minY, py <= conf.maxY,
                          pz >= conf.minZ, pz <= conf.maxZ)
        # no quantification over time for the moment
        assert(self.isInvTimeInvariant())
        assert(contract.isInvTimeInvariant())
        vcs = []
        vcs.append(     VC(prefix + "components", [sympify(self.components() == contract.components())], True) )
        if conf.enableDurationCheck:
            vcs.append( VC(prefix + "duration", [sympify(self.duration().implements(contract.duration()))], True) )
        #pre
        preExtra = And(extra.pre, extra.always)
        vcs.append( VC(prefix + "preA",   [And(preExtra, contract.preA(), Not(self.preA()))]) ) # contract.A ⇒ self.A
        vcs.append( VC(prefix + "preG",   [And(preExtra, self.preG(), Not(contract.preG()))]) ) # self.G ⇒ contract.G
        if conf.enableFPCheck:
            preHypothesis = And(preExtra,
                                self.preG(),        # strengthened by preG
                                contract.preA())    # also strengthen by assumptions as the frame shift may depends on the assumption
            vcs.append( VC(prefix + "preFP " + str(frame),  [And( preHypothesis, pointDomain, self.preFP(point), Not(contract.preFP(point)))]) ) # self.FP ⊆ contract.FP
        # inv
        invExtra = And(extra.always, contract.deTimifyFormula(self.deTimifyFormula(extra.inv)))
        vcs.append( VC(prefix + "invA",   [And(invExtra, contract.deTimifyFormula(contract.invA()), Not(self.deTimifyFormula(self.invA())))]) )
        vcs.append( VC(prefix + "invG",   [And(invExtra, self.deTimifyFormula(self.invG()), Not(contract.deTimifyFormula(contract.invG())))]) )
        if conf.enableFPCheck:
            invHypothesis = And(invExtra,
                                self.deTimifyFormula(self.invG()),
                                contract.deTimifyFormula(contract.invA()))
            vcs.append( VC(prefix + "invFP " + str(frame),  [And(invHypothesis, pointDomain, self.deTimifyFormula(self.invFP(point)), Not(contract.deTimifyFormula(contract.invFP(point))))]) )
        # post
        postExtra = And(extra.post, extra.always)
        vcs.append( VC(prefix + "postA",  [And(postExtra, contract.postA(), Not(self.postA()))]) )
        vcs.append( VC(prefix + "postG",  [And(postExtra, self.postG(), Not(contract.postG()))]) )
        if conf.enableFPCheck:
            postHypothesis = And(postExtra,
                                 self.postG(),
                                 contract.postA())
            vcs.append( VC(prefix + "postFP " + str(frame), [And(postHypothesis, pointDomain, self.postFP(point), Not(contract.postFP(point)))]) )
        # post err
        if self.postErrA() != None:
            postErrExtra = extra.always # extra.post_err ?
            vcs.append( VC(prefix + "postErrA",  [And(postExtra, contract.postErrA(), Not(self.postErrA()))]) )
            vcs.append( VC(prefix + "postErrG",  [And(postExtra, self.postErrG(), Not(contract.postErrG()))]) )
            if conf.enableFPCheck:
                postHypothesis = And(postExtra,
                                     self.postErrG(),
                                     contract.postErrA())
                vcs.append( VC(prefix + "postErrFP " + str(frame), [And(postHypothesis, pointDomain, self.postErrFP(point), Not(contract.postErrFP(point)))]) )
        return vcs

    def checkCollision(self, contract, connection, frame, extra = ExtraInfo()):
        """check if two contracts are collision free, return VCs"""
        prefix = self.name + " and " + contract.name + " collision-freedom: "
        px, py, pz = symbols('inFpX inFpY inFpZ')
        point = frame.origin.locate_new("inFp", px * frame.i + py * frame.j + pz * frame.k )
        pointDomain = And(px >= conf.minX, px <= conf.maxX,
                          py >= conf.minY, py <= conf.maxY,
                          pz >= conf.minZ, pz <= conf.maxZ)
        connectionCstrs = S.true
        for k,v in connection.items():
            connectionCstrs = And(connectionCstrs ,Eq(k,v))
        vcs = []
        #pre
        pre = And(pointDomain,
                  self.preA(),
                  self.preG(),
                  self.preFP(point),
                  contract.preA(),
                  contract.preG(),
                  contract.preFP(point),
                  connectionCstrs,
                  extra.pre,
                  extra.always)
        vcs.append( VC(prefix + "pre", [pre]) )
        #inv
        # no quantification over time for the moment
        assert(self.isInvTimeInvariant())
        assert(contract.isInvTimeInvariant())
        inv = And(pointDomain,
                  self.deTimifyFormula(self.invA()),
                  self.deTimifyFormula(self.invG()),
                  self.deTimifyFormula(self.invFP(point)),
                  contract.deTimifyFormula(contract.invA()),
                  contract.deTimifyFormula(contract.invG()),
                  contract.deTimifyFormula(contract.invFP(point)),
                  connectionCstrs,
                  self.deTimifyFormula(contract.deTimifyFormula(extra.inv)),
                  extra.always)
        vcs.append( VC(prefix + "inv",  [inv]) )
        # post
        post = And(pointDomain,
                   self.postA(),
                   self.postG(),
                   self.postFP(point),
                   contract.postA(),
                   contract.postG(),
                   contract.postFP(point),
                   connectionCstrs,
                   extra.post,
                   extra.always)
        vcs.append( VC(prefix + "post", [post]) )
        # error case is included in inv check
        return vcs

    # obstacles do not have contracts so we have a separate method to check the collision
    def checkCollisionAgainstObstacle(self, obstacle, frame, extra = ExtraInfo):
        prefix = self.name + " and " + obstacle.name() + " collision-freedom: "
        px, py, pz = symbols('inFpX inFpY inFpZ')
        point = frame.origin.locate_new("inFp", px * frame.i + py * frame.j + pz * frame.k )
        pointDomain = And(px >= conf.minX, px <= conf.maxX,
                          py >= conf.minY, py <= conf.maxY,
                          pz >= conf.minZ, pz <= conf.maxZ)
        vcs = []
        #pre
        pre = And(pointDomain,
                  self.preA(),
                  self.preG(),
                  self.preFP(point),
                  obstacle.footprint(point),
                  extra.pre,
                  extra.always)
        vcs.append( VC(prefix + "pre", [pre]) )
        #inv
        # no quantification over time for the moment
        assert(self.isInvTimeInvariant())
        inv = And(pointDomain,
                  self.deTimifyFormula(self.invA()),
                  self.deTimifyFormula(self.invG()),
                  self.deTimifyFormula(self.invFP(point)),
                  obstacle.footprint(point),
                  self.deTimifyFormula(extra.inv),
                  extra.always)
        vcs.append( VC(prefix + "inv",  [inv]) )
        # post
        post = And(pointDomain,
                   self.postA(),
                   self.postG(),
                   self.postFP(point),
                   obstacle.footprint(point),
                   extra.post,
                   extra.always)
        vcs.append( VC(prefix + "post", [post]) )
        # error case is included in inv check
        return vcs


class ComposedContract(AssumeGuaranteeContract):
    """returns a new contract which is the composition of two contracts"""

    def __init__(self, contract1, contract2, connection):
        # connection goes from outputs to inputs
        super().__init__("composition of " + contract1.name + " and " + contract2.name)
        self._contract1 = contract1
        self._contract2 = contract2
        self._connection = connection
        # sanity checks
        assert contract1.components().isdisjoint(contract2.components()), "disjoint " + contract1.name + " "  + str(contract1.components()) + " and " + contract2.name + " " + str(contract2.components())
        # duration intersect
        # FIXME put back after the thread analysis fills to duration
        # self._contract1.duration().intersect(self._contract2.duration())
        # connection is valid
        for v1, v2 in connection.items():
            assert( (v1 in contract1.outputVariables() and v2 in contract2.inputVariables()) or
                    (v1 in contract2.outputVariables() and v2 in contract1.inputVariables()) )

    def components(self):
        c1 = self._contract1.components()
        c2 = self._contract2.components()
        return c1 | c2

    #TODO for the frame pick the least least ancestor !

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

    def postErrA(self):
        #TODO errors does not propagate as the normal case and there are special checks to errors
        return None

    def postErrG(self):
        #TODO errors does not propagate as the normal case and there are special checks to errors
        return None

    def postErrFP(self, point):
        #TODO errors does not propagate as the normal case and there are special checks to errors
        return None

    def wellFormed(self, extra = ExtraInfo()):
        # this assumes that the children are well formed!
        vcs = super().wellFormed(extra)
        if conf.enableFPCheck:
            vcs.extend(self._contract1.checkCollision(self._contract2, self._connection, self.frame(), extra))
        return vcs

# Below are some generic contracts with some of the elements specified

class FpContract(AssumeGuaranteeContract):
    """A contract with only the FP specified, the rest is true"""

    def __init__(self, name, components, frame,
                 x, y, z, fp, #symbols for x, y, z, and an expression
                 duration = DurationSpec(1, 1, False)):
        self.name = name
        self._components = components
        self._frame = frame
        self._x = x
        self._y = y
        self._z = z
        self._fp = fp
        self._duration = duration

    def components(self):
        return self._components

    def frame(self):
        return self._frame

    def duration(self):
        return self._duration

    def preG(self):
        return S.true

    def fp(self, point):
        (x,y,z) = point.express_coordinates(self.frame())
        localizedFp = self._fp.subs([(self._x, x), (self._y, y), (self._z, z)])
        return localizedFp

    def preFP(self, point):
        return self.fp(point)

    def invFP(self, point):
        return self.fp(point) #TODO timify!

    def postFP(self, point):
        return self.fp(point)

    def __str__(self):
        return self.name + "{ (" + str(self._x) + ", " +  str(self._y) + ", " + str(self._z) + ") : " + str(self._fp) + " }"


class StaticContract(FpContract):

    def __init__(self, name, components, a, g, fp, duration = DurationSpec(1, 1, False)):
        super().__init__(name,
                         components, conf.worldFrame, #right now the frontend assume fp are in the world frame
                         Symbol('fpx'), Symbol('fpy'), Symbol('fpz'), fp,
                         duration)
        self._assumption = a
        self._guarantee = g

    def preA(self):
        return self._assumption

    def preG(self):
        return self._guarantee

    def invA(self):
        return self._assumption #TODO timify!

    def invG(self):
        return self._guarantee #TODO timify!

    def postA(self):
        return self._assumption

    def postG(self):
        return self._guarantee


class GContract(StaticContract):

    def __init__(self, component, expr, fp, duration = DurationSpec(1, 1, False)):
        super().__init__("GContract", {component}, S.true, expr, fp, duration)

class AContract(StaticContract):

    def __init__(self, component, expr, fp, duration = DurationSpec(1, 1, False)):
        super().__init__("AContract", {component}, expr, S.true, fp, duration)
