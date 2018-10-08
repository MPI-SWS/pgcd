import ast_inter
import sympy as sp
from spin import McMessages
from utils.vc import *
from utils.cfa import *
from utils.bbox import *
from utils.predicate_tracker import *
import logging
import spec


class GlobalModelChecking():
    
    def __init__(self, world, programs, annotations, bbox = BBox(), debug = False):
        """Checks that attached component can execute a program correctly.

        step 1: call spin to check the messages
        step 2: look at the annotation and do the compatibility check

        Keyword arguments:
        world --- the attached components
        programs --- a map from ids to programs
        annotations --- a map from program loc to formula
        bbox --- bounding box (needed for dReal constraints)
        """
        self.world = world
        self.processes = world.allProcesses()
        self.programs = programs
        self.annotations = annotations
        self.bbox = bbox
        self.debug = debug
        # logging
        self.logger = logging.getLogger(__name__)
        if debug:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('[%(name)s] %(levelname)s: %(message)s')
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)
        # cfa
        self.cfas = dict( (name, CFA(prog)) for name, prog in programs.items() )

        def logMultiline(self, level, msg):
            for l in msg.split('\n'):
                self.logger.log(level, l)

    def checkMessages(self):
        spin = McMessages(self.programs.items(), self.debug)
        ok, mps = spin.check()
        if not ok:
            raise Exception("failed to check that the communication is fine")
        else:
            return mps

    def hasAnnot(self, label):
        return label in self.annotations

    def findAnnot(self, label):
        if label in self.annotations:
            return self.annotations[label]
        else:
            self.logger.warning("could not find annotation for %s", label)
            return sp.true

    def findNextAnnot(self, label):
        openLabel = set(label)
        seenLabel = set()
        formula = sp.false #TODO ∨ or ∧ ?
        while len(openLabel) > 0:
            label = openLabel.pop()
            seenLabel.add(label)
            for n,c in self.cfas.items():
                if c.hasLabel(label):
                    for l in c.nextLabel(label):
                        if self.hasAnnot(l):
                            formula = sp.Or(formula, self.findAnnot(l)) #TODO ∨ or ∧ ?
                        elif not l in seenLabel:
                            openLabel.add(l)
        return formula
    
    def isTimeInvariant(self, formula):
        #TODO for the moment, what we have is ok but it needs to be checked
        return True

    def weakenPred(self, pred, variables):
        tracker = ProcessesPredicatesTracker(set(self.processes))
        tracker.addFormula(pred)
        tracker.relaxVariables(variables)
        return tracker.pred()

    def getMotion(self, mp, p):
        loc, name, t = mp[p.name()]
        if name.startswith("m_"):
            name = name[2:]
        motion, args = p.motionPrimitiveWithFreeParams(name) #TODO args
        return motion

    def check_motion(self, mps):
        vcs = []
        px, py, pz = sp.symbols('inFpX inFpY inFpZ')
        pointDomain = self.bbox.contains(px, py, pz)
        # a point for the footprint
        point = self.world.frame().origin.locate_new("inFp", px * self.world.frame().i + py * self.world.frame().j + pz * self.world.frame().k )
        # processes and abstraction
        for p in self.processes:
            self.logger.debug("correctness of footprint abstraction for %s", p.name())
            point = p.frame().origin.locate_new("inFp", px * p.frame().i + py * p.frame().j + pz * p.frame().k )
            hypotheses = sp.And(p.invariant(), pointDomain, p.ownResources(point))
            concl = p.abstractResources(point)
            vcs.append( VC("correctness of footprint abstraction for " + p.name(), [sp.And(hypotheses, sp.Not(concl))]) )
        assumptions = sp.And(*[ p.invariant() for p in self.processes ]) #TODO add the connection as ==
        for mp in mps:
            self.logger.info("mp: %s", mp) # mp is map from name to loc, mp, time
            # pre
            annotAtPre = sp.And(*[ self.findAnnot(loc) for name, (loc, mp, t) in mp.items() ])
            changedVariables = set()
            for p in self.processes:
                motion = self.getMotion(mp, p)
                #pre spec implies precondition
                vcs.append( VC("precondition of " + motion.name() + " for " + p.name() + " @ " + str(mp), [sp.And(assumptions, annotAtPre, sp.Not(motion.pre()))]) )
                #pre spec has resources
                fpCoarse = sp.And(assumptions, annotAtPre, pointDomain, p.abstractResources(point), sp.Not(motion.preFP(point)))
                fpFine = sp.And(assumptions, annotAtPre, pointDomain, p.ownResources(point), sp.Not(motion.preFP(point)))
                vcs.append( VC("pre resources of " + motion.name() + " for " + p.name() + " @ " + str(mp), [fpCoarse, fpFine]) )
                #pre no collision
                for p2 in self.processes:
                    if p.name() < p2.name():
                        motion2 = self.getMotion(mp, p2)
                        fs = [sp.And(assumptions, annotAtPre, pointDomain, motion.preFP(point), motion2.preFP(point))]
                        vcs.append( VC("no collision in precondition for " + p.name() + " and " + p2.name() + " @ " + str(mp), fs) )
                changedVariables.update(motion.modifies())
            frame = self.weakenPred(annotAtPre, changedVariables) 
            self.logger.debug("changedVariables: %s", changedVariables)
            self.logger.debug("annotAtPre: %s", annotAtPre)
            self.logger.debug("frame: %s", frame)
            # inv
            inv = frame
            for p in self.processes:
                self.logger.debug("invariant (1) for process %s", p.name())
                motion = self.getMotion(mp, p)
                f = motion.inv()
                assert(self.isTimeInvariant(f))
                inv = sp.And(inv, spec.deTimifyFormula(p.variables(), f))
            #inv is sat
            vcs.append( VC("inv is sat @ " + str(mp), [And(assumptions, inv)], True) )
            #inv no collision
            for p in self.processes:
                self.logger.debug("invariant (2) for process %s", p.name())
                motion = self.getMotion(mp, p)
                f1 = motion.invFP(point)
                assert(self.isTimeInvariant(f1))
                f1 = spec.deTimifyFormula(p.variables(), f1)
                for p2 in self.processes:
                    if p.name() < p2.name():
                        motion2 = self.getMotion(mp, p2)
                        f2 = motion2.invFP(point)
                        assert(self.isTimeInvariant(f2))
                        f2 = spec.deTimifyFormula(p2.variables(), f2)
                        fs = [sp.And(assumptions, inv, pointDomain, f1, f2)]
                        vcs.append( VC("no collision in inv for " + p.name() + " and " + p2.name() + " @ " + str(mp), fs) )
                #process resources are included in invFP
                fpCoarse = sp.And(assumptions, inv, pointDomain, p.abstractResources(point), sp.Not(f1))
                fpFine = sp.And(assumptions, inv, pointDomain, p.ownResources(point), sp.Not(f1))
                vcs.append( VC("inv resources of " + motion.name() + " for " + p.name() + " @ " + str(mp), [fpCoarse, fpFine]) )
            # post
            annotAtPost = sp.And(*[ self.findNextAnnot(loc) for name, (loc, mp, t) in mp.items() ])
            #post postconditions is sat
            post = frame
            for p in self.processes:
                self.logger.debug("post (1) for process %s", p.name())
                motion = self.getMotion(mp, p)
                post = sp.And(post, motion.post())
            vcs.append( VC("post is sat @ " + str(mp), [sp.And(assumptions, post)], True) )
            #post postconditions implies spec
            vcs.append( VC("post implies annot @ " + str(mp), [sp.And(assumptions, post, sp.Not(annotAtPost))]) )
            #post no collision
            for p in self.processes:
                self.logger.debug("post (2) for process %s", p.name())
                motion = self.getMotion(mp, p)
                f1 = motion.postFP(point)
                for p2 in self.processes:
                    if p.name() < p2.name():
                        self.logger.debug("post (3) for process %s", p2.name())
                        motion2 = self.getMotion(mp, p2)
                        f2 = motion2.postFP(point)
                        fs = [sp.And(assumptions, post, pointDomain, f1, f2)]
                        vcs.append( VC("no collision in post for " + p.name() + " and " + p2.name() + " @ " + str(mp), fs) )
                    #process resources are included in postFP
                    fpCoarse = sp.And(assumptions, post, pointDomain, p.abstractResources(point), sp.Not(f1))
                    fpFine = sp.And(assumptions, post, pointDomain, p.ownResources(point), sp.Not(f1))
                    vcs.append( VC("post resources of " + motion.name() + " for " + p.name() + " @ " + str(mp), [fpCoarse, fpFine]) )
        for vc in vcs:
            if not vc.discharge(timeout = 300, debug = self.debug):
                raise Exception("failed to check " + str(vc))

    def check(self):
        mps = self.checkMessages()
        self.logger.info("checkMessages returned %s", mps)
        self.check_motion(mps)
