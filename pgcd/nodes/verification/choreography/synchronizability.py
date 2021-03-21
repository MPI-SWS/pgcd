from ast_chor import *
from utils.fixed_point import *
from spec.time import DurationSpec
from copy import copy
import logging

log = logging.getLogger("Synchronizability")

# Synchronizability check

class SyncTracker:

    def __init__(self, stack, process_set):
        self.undef = True
        self.processes = process_set
        self.stack = stack
        self.popAfterMessage = False
        self.duration = dict()
        self.hasSender = False
        for p in self.processes:
            self.duration[p] = DurationSpec(0, 0, False)

    def copy(self):
        cpy = SyncTracker(copy(self.stack), self.processes)
        cpy.undef = self.undef
        cpy.duration = copy(self.duration)
        return cpy

    def inSync(self):
        assert not self.undef
        return self.processes.size() == 1 or all(self.duration[p].max == 0 for p in self.processes)

    def restrict(self, process_set):
        if not process_set.issubset(process_set):
            log.warning("restricting to different %s -> %s", self.processes, process_set)
        self.processes = process_set
        toRemove = []
        for p in self.duration.keys():
            if p not in self.processes:
                toRemove.append(p)
        for p in toRemove:
            del self.duration[p]

    def push(self):
        if not self.undef:
            self.stack.push(DurationSpec(0, 0, False))

    def pop(self):
        if not self.undef:
            top = self.stack.pop()
            self.stack[-1] = self.stack[-1].concat(top)

    def accelerate(self):
        if not self.undef:
            # a single process does not send messages so the stack stays unchanged
            # an alternative is to change the stack on motions when there is only one process
            if len(self.processes) == 1:
                for p in self.processes:
                    self.duration[p].max = float('inf')
            else:
                self.stack[-1].max = float('inf')

    def merge(self, oldDestTracker):
        if self.undef:
            self.undef = oldDestTracker.undef
            self.stack = copy(oldDestTracker.stack)
            self.popAfterMessage = oldDestTracker.popAfterMessage
            self.duration = copy(oldDestTracker.duration)
        elif not oldDestTracker.undef:
            for p in self.processes:
                self.duration[p].min = min(oldDestTracker.duration[p].min, self.duration[p].min)
                self.duration[p].max = max(oldDestTracker.duration[p].max, self.duration[p].max)
            self.stack[-1].min = min(oldDestTracker.stack[-1].min, self.stack[-1].min)
            self.stack[-1].max = max(oldDestTracker.stack[-1].max, self.stack[-1].max)

    def join(self, oldDestTracker):
        assert(self.processes.issubset(oldDestTracker.processes))
        self.popAfterMessage = True
        if self.undef:
            self.undef = oldDestTracker.undef
            self.processes = oldDestTracker.processes
            self.stack = copy(oldDestTracker.stack)
            self.duration = copy(oldDestTracker.duration)
        else:
            if self.hasSender:
                self.stack = oldDestTracker.stack
            else:
                # TODO we should keep the other stack to do a proper sync check !!
                pass
            for p in oldDestTracker.processes:
                if p not in self.processes:
                    self.duration[p] = oldDestTracker.duration[p].copy()
            self.processes = oldDestTracker.processes

    def __eq__(self, tracker):
        if isinstance(tracker, SyncTracker):
            return self.undef == tracker.undef and \
                   self.processes == tracker.processes and \
                   self.stack == tracker.stack and \
                   self.popAfterMessage == tracker.popAfterMessage and \
                   all(self.duration[p] == tracker.duration[p] for p in self.processes)
        else:
            return False

    def __str__(self):
        if self.undef:
            return "SyncTracker: -"
        else:
            acc = "SyncTracker:\n  stack: " + ', '.join(str(e) for e in self.stack)
            for p in self.processes:
                acc = acc + "\n  " + p + " -> " + str(self.duration[p])
            acc = acc + "\n  popAfterMessage: " + str(self.popAfterMessage)
            return acc

class Synchronizability(FixedPointDataflowAnalysis):

    # choice
    # merge
    # fork
    # join
    # motion
    # messages
    # acceleration

    def __init__(self, chor, env, threads, minSender):
        super().__init__(chor, env.allProcesses(), True)
        self.nodeToProcesses = threads #TODO could be already filled in chor itself
        self.nodeToMinSender = minSender
        self.s2n = self.chor.mk_state_to_node()

    def getMotionDuration(self, process, motionName, motionArgs):
        p = self.chor.getProcess(process)
        mp = p.motionPrimitive(motionName, *motionArgs)
        return mp.duration()

    def canReach(self, state1, state2):
        frontier = {state1}
        visited = set()
        while frontier != set():
            state = frontier.pop()
            if state == state2:
                return True
            else:
                visited.add(state)
                node = self.s2n[state]
                for s in node.end_state:
                    if s not in visited:
                        frontier.add(s)
        return False

    def initialValue(self, state, node):
        stack = [DurationSpec(0,0,False)]
        procs = self.nodeToProcesses[state]
        t = SyncTracker(stack, procs)
        if state == self.chor.start_state:
            t.undef = False
        return t

    def motion(self, tracker, motions):
        if not tracker.undef:
            for m in motions:
                d = self.getMotionDuration(m.id, m.mp_name, m.mp_args)
                tracker.duration[m.id] = tracker.duration[m.id].concat(d)
        return tracker

    def message(self, tracker, node):
        if not tracker.undef:
            tracker.stack[-1] = tracker.stack[-1].concat(tracker.duration[node.sender])
            tracker.duration[node.sender] = DurationSpec(0,0,False)
            tracker.duration[node.receiver] = DurationSpec(0,0,False)
            if tracker.popAfterMessage == True:
                tracker.pop()
                tracker.popAfterMessage == False
        return tracker

    def _merge(self, pred, succ):
        assert self.forward
        trackerSrc = self.state_to_element[pred]
        trackerDst = self.state_to_element[succ]
        tracker = trackerSrc.copy()
        if self.canReach(succ, pred):
            tracker.accelerate()
        return self._goesTo(tracker, pred, succ)

    def _fork(self, pred, succ):
        assert self.forward
        trackerSrc = self.state_to_element[pred]
        trackerDst = self.state_to_element[succ]
        tracker = trackerSrc.copy()
        tracker.restrict(trackerDst.processes)
        return self._goesTo(tracker, pred, succ)

    def isMinSender(self, state):
        # a bit weird but the min sender analysis comes after sending
        if self.nodeToMinSender[state] == None:
            node = self.state_to_node[state]
            s2 = node.end_state[0]
            return node.sender == self.nodeToMinSender[s2]
        else:
            return False

    def hasNextMinSender(self, state):
        node = self.state_to_node[state]
        if isinstance(node, Message):
            return self.isMinSender(state)
        else:
            return self.hasNextMinSender(node.end_state[0])

    def _goesTo(self, tracker, pred, succ):
        assert self.forward
        node = self.state_to_node[pred]
        if isinstance(node, Join):
            if self.hasNextMinSender(pred):
                tracker.hasNextMinSender = True
            trackerOld = self.state_to_element[succ]
            tracker.join(trackerOld)
            self.state_to_element[succ] = tracker
            res = not trackerOld == tracker
            if log.isEnabledFor(logging.DEBUG) and res:
                log.debug("changed %s %s to %s", state, node, tracker)
            return res
        else:
            super()._goesTo(tracker, pred, succ)

    def check(self):
        log.debug("unique minimal sender")
        self.perform()
        node_to_tracker = self.result()
        state_to_node = self.chor.mk_state_to_node()
        for state in node_to_tracker.keys():
            node = state_to_node[state]
            t = node_to_tracker[state]
            if isinstance(node, Checkpoint) or isinstance(node, Motion) or isinstance(node, Fork):
                # checkpoints, motion, fork occurs only when the processes are in sync
                t = node_to_tracker[state]
                assert t.inSync, "process not in sync: " + str(node) + " " + str(t)
            elif isinstance(node, Message):
                # print("===================")
                # print(node)
                # print("-------------------")
                # print(t)
                # print("-------------------")
                # print(node_to_tracker[node.end_state[0]])
                # print("-------------------")
                # sender should be uninterruptible
                assert not t.duration[node.sender].interruptible, "interruptible process sending: " + str(node) + " " + str(t)
                # check that the minimal sender is the non-interruptible process, all the other muse be interruptible
                if self.isMinSender(state):
                    for p in t.processes:
                        if p != node.sender:
                            # special case: processes start in sync
                            assert t.duration[p] == DurationSpec(0,0,False) or \
                                   t.duration[p].interruptible, \
                                   "more than one uninterruptible process: " + str(node) + " " + str(t)
                        # check that the timing are compatible
                        assert t.duration[p].min <= t.duration[node.sender].min, "incompatible durations: " + str(node) + " " + str(t)
                        assert t.duration[p].max >= t.duration[node.sender].max, "incompatible durations: " + str(node) + " " + str(t)
            elif isinstance(node, Join):
                #TODO the two stack must match
                pass
        return node_to_tracker
