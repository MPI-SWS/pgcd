from abc import ABC, abstractmethod
from ast_chor import *
from typing import List
import logging

log = logging.getLogger("FixedPoint")

class FixedPointDataflowAnalysis(ABC):

    def __init__(self, chor, processes, forward = True):
        self.chor = chor
        self.state_to_node = chor.mk_state_to_node()
        if type(processes) is set:
            self.processes = processes
        else:
            self.processes = set(processes)
        self.state_to_element = {}
        self._mergeMap = {}
        self.forward = forward
        self.done = False

    ##################################
    ## Begin operations to override ##
    ## the default methods are for  ##
    ## a forward analysis.          ##
    ##################################

    # the element must provide a `copy`, `equals`, `merge`, and `join` methods

    @abstractmethod
    def initialValue(self, state, node):
        pass

    def motion(self, tracker, motions):
        return tracker

    def message(self, tracker, node):
        return tracker

    def guard(self, tracker, guard):
        return tracker

    def merge(self, tracker):
        return tracker

    def fork(self, tracker):
        return tracker

    def join(self, tracker):
        return tracker

    def checkpoint(self, tracker):
        return tracker

    def _motion(self, pred, motions, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = self.motion(trackerSrc.copy(), motions)
        return self._goesTo(tracker, pred, succ)

    def _message(self, pred, node, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = self.message(trackerSrc.copy(), node)
        return self._goesTo(tracker, pred, succ)

    def _guard(self, pred, guard, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = self.guard(trackerSrc.copy(), guard)
        return self._goesTo(tracker, pred, succ)

    def _merge(self, pred, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = self.merge(trackerSrc.copy())
        return self._goesTo(tracker, pred, succ)

    def _fork(self, pred, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = self.fork(trackerSrc.copy())
        return self._goesTo(tracker, pred, succ)

    def _join(self, pred, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = self.join(trackerSrc.copy())
        return self._goesTo(tracker, pred, succ)

    def _checkpoint(self, pred, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = self.checkpoint(trackerSrc.copy())
        return self._goesTo(tracker, pred, succ)

    ################################
    ## End operations to override ##
    ################################

    def _goesTo(self, tracker, pred, succ):
        state = succ if self.forward else pred
        node = self.state_to_node[pred]
        trackerOld = self.state_to_element[state]
        if self.forward and isinstance(node, Merge):
            tracker.merge(trackerOld)
        elif self.forward and isinstance(node, Join):
            tracker.join(trackerOld)
        elif not self.forward and isinstance(node, GuardedChoice):
            tracker.merge(trackerOld)
        elif not self.forward and isinstance(node, Fork):
            tracker.join(trackerOld)
        self.state_to_element[state] = tracker
        res = not trackerOld.equals(tracker)
        if log.isEnabledFor(logging.DEBUG) and res:
            log.debug("changed %s %s to %s", state, node, tracker)
        return res

    def processForMotion(self, motion):
        candidates = [ p for p in self.processes if p == motion.id or p.name() == motion.id ]
        assert(len(candidates) <= 1)
        if len(candidates) == 0:
            return None
        elif len(candidates) == 1:
            return candidates.pop()
        else:
            raise Exception("more than one process for motion primitive " + motion.id + " -> " + str(candidates))

    def result(self):
        assert self.done
        return self.state_to_element

    def perform(self):
        assert not self.done
        # initialization
        for s in self.state_to_node.keys():
            n = self.state_to_node[s]
            tracker = self.initialValue(s, n)
            self.state_to_element[s] = tracker
            log.debug("initial %s %s", s, tracker)
        #loop
        changed = True
        counter = 0
        while changed:
            if log.isEnabledFor(logging.DEBUG):
                log.debug("\n\n==========================")
                log.debug("iteration %s", counter)
                for state in self.state_to_node.keys():
                    log.debug("%s %s", state, self.state_to_node[state])
                    log.debug("%s", self.state_to_element[state])
                log.debug("--------------------------")
            counter = counter + 1
            changed = False
            # first the non-merge
            for state in self.state_to_node.keys():
                node = self.state_to_node[state]
                log.debug("processing %s %s %s", state, node, self.state_to_element[state])
                if isinstance(node, Message):
                    succ = node.end_state[0]
                    res = self._message(state, node, succ)
                    changed = changed or res
                elif isinstance(node, GuardedChoice):
                    for gs in node.guarded_states:
                        guard = gs.expression
                        res = self._guard(state, guard, gs.id)
                        changed = changed or res
                elif isinstance(node, Fork):
                    for succ in node.end_state:
                        res = self._fork(state, succ)
                        changed = changed or res
                elif isinstance(node, Motion):
                    succ = node.end_state[0]
                    res = self._motion(state, node.motions, succ)
                    changed = changed or res
                elif isinstance(node, Merge):
                    succ = node.end_state[0]
                    res = self._merge(state, succ)
                    changed = changed or res
                elif isinstance(node, Join):
                    succ = node.end_state[0]
                    res = self._join(state, succ)
                    changed = changed or res
                elif isinstance(node, Checkpoint):
                    succ = node.end_state[0]
                    res = self._checkpoint(state, succ)
                    changed = changed or res
                elif isinstance(node, End):
                    pass
                else:
                    raise Exception("??? " + str(node))
        self.done= True
