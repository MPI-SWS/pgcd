from verification.choreography.ast_chor import *
from verification.utils.fixed_point import *
from copy import copy
import logging

log = logging.getLogger("MinimalSender")
# check unique minimal sender and all processes in thread receive

class MinimalSenderTracker:

    def __init__(self, process_set = set()):
        # has not seen anything before
        self.undef = True
        # all the processes in the current thread
        self.processes = process_set
        # minimal sender
        self.minimalSender = set()
        # processes that have received since the last motion
        self.receivers = set()
        # because our syntax decouple choice and communication
        self.needMessage = False

    def copy(self):
        cpy = MinimalSenderTracker()
        cpy.undef = self.undef
        cpy.processes = self.processes
        cpy.minimalSender = copy(self.minimalSender)
        cpy.receivers = copy(self.receivers)
        cpy.needMessage = self.needMessage
        return cpy

    def restrict(self, process_set):
        if not process_set.issubset(process_set):
            log.warning("restricting to different %s -> %s", self.processes, process_set)
        self.processes = process_set
        self.minimalSender &= process_set
        self.receivers &= process_set

    def take(self, tracker):
        self.undef = tracker.undef
        self.processes = tracker.processes
        self.minimalSender = copy(tracker.minimalSender)
        self.receivers = copy(tracker.receivers)
        self.needMessage = tracker.needMessage

    def merge(self, oldDestTracker):
        if self.undef:
            self.take(oldDestTracker)
        elif not oldDestTracker.undef:
            self.minimalSender |= oldDestTracker.minimalSender
            self.receivers &= oldDestTracker.receivers
            self.needMessage |= oldDestTracker.needMessage

    def join(self, oldDestTracker):
        if self.undef:
            self.take(oldDestTracker)
        elif not oldDestTracker.undef:
            self.processes = oldDestTracker.processes
            self.minimalSender |= oldDestTracker.minimalSender
            self.receivers |= oldDestTracker.receivers
            self.needMessage |= oldDestTracker.needMessage
        else:
            self.processes = oldDestTracker.processes

    def __eq__(self, tracker):
        if isinstance(tracker, MinimalSenderTracker):
            return self.undef == tracker.undef and \
                   self.minimalSender == tracker.minimalSender and \
                   self.processes == tracker.processes and \
                   self.receivers == tracker.receivers and \
                   self.needMessage == tracker.needMessage
        else:
            return False

    def __str__(self):
        if self.undef:
            return "MinimalSenderTracker: -"
        else:
            return "MinimalSenderTracker: " + str(self.minimalSender) + ", " + str(self.receivers) + ", " + str(self.needMessage)

class UniqueMinimalSender(FixedPointDataflowAnalysis):

    # Summary
    # * choice: copy
    # * fork: copy and restrict
    # * merge: (overappox) union minimalSender, intersect receivers
    # * join: union
    # * motion: reset sets, set flag
    # * messages: move processes, reset flag


    # threads is a map of nodes to processes (result of ThreadChecks)
    def __init__(self, chor, env, threads):
        super().__init__(chor, env.allProcesses(), True)
        self.nodeToProcesses = threads

    def initialValue(self, state, node):
        t = MinimalSenderTracker(self.nodeToProcesses[state])
        if state == self.chor.start_state:
            t.undef = False
        return t

    def guard(self, tracker, guard):
        tracker.needMessage = True
        return tracker

    def _fork(self, pred, succ):
        assert self.forward
        trackerSrc = self.state_to_element[pred]
        trackerDst = self.state_to_element[succ]
        tracker = trackerSrc.copy()
        tracker.restrict(trackerDst.processes)
        return self._goesTo(tracker, pred, succ)

    def motion(self, tracker, motions):
        tracker.minimalSender = set()
        tracker.receivers = set()
        return tracker

    def message(self, tracker, node):
        tracker.needMessage = False
        if node.sender not in tracker.receivers:
            tracker.minimalSender.add(node.sender)
        tracker.receivers.add(node.receiver)
        return tracker

    def check(self):
        log.debug("unique minimal sender")
        self.perform()
        node_to_tracker = self.result()
        state_to_node = self.chor.mk_state_to_node()
        for (state, t) in node_to_tracker.items():
            node = state_to_node[state]
            assert len(t.minimalSender) <= 1, "more than one minimalSender " + state + ", " + str(t.minimalSender)
            # this get check with the sync
            #if isinstance(node, Checkpoint):
            #    assert (t.receivers + t.sender) == t.processes, "checkpoint when processes not in sync " + state + ", " + str(t.receivers)
            if isinstance(node, Motion):
                assert (len(t.processes) == 1 or not t.needMessage) or (t.receivers == t.processes), \
                       "motions when processes not in sync " + state + ", " + str(t.receivers)
            #TODO assertion about needMessage ??
        return node_to_tracker

