from abc import ABC, abstractmethod
from ast_chor import *
import copy

class FixedPointDataflowAnalysis(ABC):

    def __init__(self, chor, world):
        self.chor = chor
        self.state_to_node = chor.mk_state_to_node()
        self.world = world
        self.processes = world.allProcesses()
        self.node_to_element = {}
        self._mergeMap = {}
        self.done = False
    
    ##################################
    ## Begin operations to override ##
    ##################################

    @abstractmethod
    def startElement(self, node):
        pass
    
    @abstractmethod
    def defaultElement(self, node):
        pass
    
    def _guard(self, pred, guard, succ):
        trackerSrc = self.node_to_element[pred]
        tracker = trackerSrc.copy()
        return self._goesInto(tracker, succ)

    def _motion(self, pred, motions, succ):
        trackerSrc = self.node_to_element[pred]
        tracker = trackerSrc.copy()
        return self._goesInto(tracker, succ)

    def _message(self, pred, succ):
        trackerSrc = self.node_to_element[pred]
        tracker = trackerSrc.copy()
        return self._goesInto(tracker, succ)

    def _fork(self, pred, succ):
        trackerSrc = self.node_to_element[pred]
        tracker = trackerSrc.copy()
        return self._goesInto(tracker, succ)

    def _merge(self, node):
        tracker = self.defaultElement(node)
        for t in self._mergeMap[node]:
            tracker.merge(t)
        self._mergeMap[node] = []
        trackerOld = self.node_to_element[node]
        self.node_to_element[node] = tracker
        return not trackerOld.equals(tracker)
    
    def _join(self, node):
        tracker = self.defaultElement(node) #TODO !?!?
        for t in self._mergeMap[node]:
            tracker.merge(t) #TODO !?!?
        self._mergeMap[node] = []
        trackerOld = self.node_to_element[node]
        self.node_to_element[node] = tracker
        return not trackerOld.equals(tracker)
    
    ################################
    ## End operations to override ##
    ################################
    
    def _goesInto(self, tracker, succ):
        if isinstance(succ, Merge) or isinstance(succ, Join):
            succ = self.state_to_node[succ.end_state[0]]
            self._mergeMap[succ].append(tracker)
        else:
            trackerOld = self.node_to_element[succ]
            self.node_to_element[succ] = tracker
            return not trackerOld.equals(tracker)

    def processForMotion(self, motion):
        candidates = [ p for p in self.processes if p.name() == motion.id ]
        assert(len(candidates) <= 1)
        if len(candidates) == 0:
            return None
        elif len(candidates) == 1:
            return candidates.pop()
        else:
            raise Exception("more than one process for motion primitive " + motion.id + " -> " + str(candidates))

    def result(self):
        assert self.done
        return self.node_to_element

    def perform(self, debug = False):
        assert not self.done
        # initialization
        for s in self.state_to_node.keys():
            n = self.state_to_node[s]
            if s == self.chor.start_state:
                self.node_to_element[n] = self.startElement(n)
            else:
                self.node_to_element[n] = self.defaultElement(n)
            self._mergeMap[n] = []
        #loop
        changed = True
        counter = 0
        while changed:
            counter = counter + 1
            if debug:
                print("")
                print("")
                print("")
                print("==========================")
                print("==========================")
                print("iteration " + str(counter))
                for node in self.state_to_node.values():
                    print(node)
                    print(self.node_to_element[node])
                print("==========================")
            changed = False
            # first the non-merge
            for node in self.state_to_node.values():
                if debug:
                    print("processing " + str(node))
                if isinstance(node, Message):
                    succ = self.state_to_node[node.end_state[0]]
                    res = self._message(node, succ)
                    if debug and res:
                        print("changed: " + str(node))
                    changed = changed or res
                elif isinstance(node, GuardedChoice):
                    for gs in node.guarded_states:
                        guard = gs.expression
                        succ = self.state_to_node[gs.id]
                        res = self._guard(node, guard, succ)
                        if debug and res:
                            print("changed: " + str(node))
                        changed = changed or res
                elif isinstance(node, Fork):
                    for s in node.end_state:
                        succ = self.state_to_node[s]
                        res = self._fork(node, succ)
                        if debug and res:
                            print("changed: " + str(node))
                        changed = changed or res
                elif isinstance(node, Motion):
                    processes = []
                    mps = []
                    for motion in node.motions:
                        proc = self.processForMotion(motion)
                        mpName = motion.mp_name
                        mpArgs = motion.mp_args
                        mp = proc.motionPrimitive(mpName, *mpArgs)
                        processes.append(proc)
                        mps.append(mp)
                    #TODO take the thread partition into account for restricting the elements!
                    assert set(processes) == set(self.processes), str(processes) + " != " + str(set(self.processes))
                    succ = self.state_to_node[node.end_state[0]]
                    res = self._motion(node, mps, succ)
                    if debug and res:
                        print("changed: " + str(node))
                    changed = changed or res
            #finish the merge/join
            merged = set()
            for node in self.state_to_node.values():
                if isinstance(node, Merge):
                    succ = self.state_to_node[node.end_state[0]]
                    if not succ in merged:
                        merged.add(succ)
                        res = self._merge(succ)
                        if debug and res:
                            print("changed: " + str(node))
                        changed = changed or res
                elif isinstance(node, Join):
                    succ = self.state_to_node[node.end_state[0]]
                    if not succ in merged:
                        merged.add(succ)
                        res = self._join(succ)
                        if debug and res:
                            print("changed: " + str(node))
                        changed = changed or res
        self.done= True
