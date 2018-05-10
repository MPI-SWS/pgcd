from ast_chor import *
from utils.fixed_point import *
import copy

# thread partition and correctness checks

class ThreadTracker:

    def __init__(self, stack = [], process_set = set()):
        self.stack = stack
        self.processes = process_set
        self.seen_mp = False

    def copy(self):
        return copy.copy(self)

    def push(self, nodeFrom, nodeTo):
        self.stack.append( (node, nodeFrom, nodeTo) )
    
    def pop(self, node, idx):
        return self.pop

    def addProcess(self, proc):
        self.processes = self.processes | {proc}
    
    def clearProcesses(self):
        self.processes = set()

    def lastFork(self):
        return self.stack[-1]

    def merge(self, tracker):
        pass

    def join(self, tracker):
        self.process = self.processes | tracker.processes

    def equals(self, tracker):
        return self.stack == tracker.stack and self.processes == tracker.processes and self.seen_mp == tracker.seen_mp

    def __str__(self):
        return "ThreadTracker: " + str(self.stack) + ", " + str(self.processes) + ", " + str(self.seen_mp)


class ComputeThreads(FixedPointDataflowAnalysis):

    def getProcesses(self):
        return { p if isinstance(p, str) else p.name() for p in self.processes }

    def initialValue(self, node):
        if node.start_state[0] == self.chor.start_state:
            return ThreadTracker([], self.getProcesses() )
        else:
            return ThreadTracker([], set())


    def _motion(self, pred, motions, succ):
        trackerSrc = self.node_to_element[pred]
        tracker = trackerSrc.copy()
        for mp in motions:
            tracker.addProcess(mp.id)
        tracker.seen_mp = True
        return self._goesInto(tracker, succ)

    def _fork(self, pred, succ):
        trackerSrc = self.node_to_element[pred]
        tracker = trackerSrc.copy()
        tracker.push(pred, succ)
        tracker.clearProcesses()
        tracker.seen_mp = False
        return self._goesInto(tracker, succ)
    
    def _join(self, node):
        tracker = None
        for t in self._mergeMap[node]:
            if tracker == None:
                tracker = t.copy()
            else:
                tracker.join(t)
        self._mergeMap[node] = []
        if len(tracker.stack) > 0:
            forkNode = tracker.pop()[0]
            if not tracker.seen_mp:
                trackerFork = self.node_to_element[forkNode]
                tracker.processes = forkNode.processes
        trackerOld = self.node_to_element[node]
        self.node_to_element[node] = tracker
        return not trackerOld.equals(tracker)


class ThreadChecks():

    def __init__(self, chor, processes):
        self.chor = chor
        self.processes = processes

    def perform(self, debug = False):
        if debug:
            print("thread correctness and partition")
        cp = ComputeThreads(self.chor, self.processes)
        cp.perform(debug)
        node_to_trackers = cp.result()
        state_to_node = self.chor.mk_state_to_node()
        def getPreds(node):
            return [ node_to_trackers[state_to_node[s]] for s in node.start_state ]
        for node in node_to_trackers.keys():
            if isinstance(node, End):
                t = node_to_trackers[node]
                assert t.stack == [], "ending in the middle of a fork: " + str(node) + " " + str(t)
            elif isinstance(node, Merge):
                t = node_to_trackers[node]
                preds = getPreds(node)
                assert all( p.equals(t) for p in preds )
            elif isinstance(node, Fork):
                preds = getPreds(node)
                rep = preds[0]
                forkNode = rep.lastFork()[0]
                assert all(p.stack == rep.stack for p in preds), "joining thread from different fork"
                assert all(p.seen_mp == rep.seen_mp for p in preds), "joining thread with and without having seen an motion primitive"
                assert all(any(p.lastFork()[1] == s for p in preds) for s in forkNode.end_state), "not every thread is joined"
                assert all(p.lastFork()[1] in forkNode.end_state), "every thread joining from ???"
                assert all(p == rep or p.lastFork()[1] != rep.lastFork()[1]), "joining same thread multiple time"
                assert all(p == rep or len(p.processes & rep.processes) == 0 for p in preds), "thread partition is not respected"
        #TODO check that every thread is eventually joined
        return node_to_trackers
