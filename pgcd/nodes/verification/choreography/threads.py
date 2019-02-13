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
        cpy = ThreadTracker()
        cpy.stack = copy.copy(self.stack)
        cpy.processes = self.processes
        cpy.seen_mp = self.seen_mp
        return cpy

    def push(self, nodeFrom, nodeTo):
        if any( n == nodeFrom for n,t in self.stack ):
            raise Exception("fork again on " + nodeFrom + " before joining.")
        self.stack.append( (nodeFrom, nodeTo) )
    
    def pop(self):
        return self.stack.pop()

    def addProcess(self, proc):
        self.processes = self.processes | {proc}
    
    def clearProcesses(self):
        self.processes = set()

    def lastFork(self):
        return self.stack[-1]

    def merge(self, tracker):
        self.join(tracker) #TODO better

    def join(self, tracker):
        if len(self.stack) < len(tracker.stack) or (len(self.stack) > 0 and len(self.stack) == len(tracker.stack) and self.lastFork() > tracker.lastFork()):
            self.stack = tracker.stack
        self.processes = self.processes | tracker.processes
        self.seen_mp = self.seen_mp or tracker.seen_mp #TODO better

    def equals(self, tracker):
        return self.stack == tracker.stack and self.processes == tracker.processes and self.seen_mp == tracker.seen_mp

    def __str__(self):
        return "ThreadTracker: " + str(self.stack) + ", " + str(self.processes) + ", " + str(self.seen_mp)


class ComputeThreads(FixedPointDataflowAnalysis):

    def __init__(self, chor, processes, debug = False):
        super().__init__(chor, processes, True, debug)

    def getProcesses(self):
        return { p if isinstance(p, str) else p.name() for p in self.processes }

    def initialValue(self, state, node):
        if state == self.chor.start_state:
            return ThreadTracker([], self.getProcesses() )
        else:
            return ThreadTracker([], set())

    def motion(self, tracker, motions):
        for mp in motions:
            tracker.addProcess(mp.id)
        tracker.seen_mp = True
        return tracker
    
    def join(self, tracker):
        if len(tracker.stack) > 0:
            forkNode = tracker.pop()[0]
            if not tracker.seen_mp:
                trackerFork = self.state_to_element[forkNode]
                tracker.processes = forkNode.processes
        return tracker

    def _fork(self, pred, succ):
        trackerSrc = self.state_to_element[pred if self.forward else succ]
        tracker = trackerSrc.copy()
        tracker.push(pred, succ)
        tracker.clearProcesses()
        tracker.seen_mp = False
        return self._goesTo(tracker, pred, succ)


class ThreadChecks():

    def __init__(self, chor, processes):
        self.chor = chor
        self.processes = processes

    def perform(self, debug = False):
        if debug:
            print("thread correctness and partition")
        cp = ComputeThreads(self.chor, self.processes, debug)
        cp.perform()
        node_to_trackers = cp.result()
        state_to_node = self.chor.mk_state_to_node()
        def getPreds(node):
            return [ node_to_trackers[s] for s in node.start_state ]
        for state in node_to_trackers.keys():
            node = state_to_node[state]
            if isinstance(node, End):
                t = node_to_trackers[state]
                assert t.stack == [], "ending in the middle of a fork: " + str(node) + " " + str(t)
                assert t.processes == cp.getProcesses(), "not all the processes got joined " + str(node) + " " + str(t)
            elif isinstance(node, Merge):
                t = node_to_trackers[state]
                preds = getPreds(node)
                assert all( p.equals(t) for p in preds )
            elif isinstance(node, Join):
                preds = getPreds(node)
                rep = preds[0]
                forkNode = rep.lastFork()[0]
                assert all(len(p.stack) == len(rep.stack) for p in preds), "joining thread from different fork"
                assert all(len(p.stack) == 0 or p.stack[:-1] == rep.stack[:-1] for p in preds), "joining thread from different fork"
                assert all(len(p.stack) == 0 or p.lastFork()[0] == rep.lastFork()[0] for p in preds), "joining thread from different fork"
                assert all(p.seen_mp == rep.seen_mp for p in preds), "joining thread with and without having seen an motion primitive"
                assert all(any(p.lastFork()[1] == s for p in preds) for s in state_to_node[forkNode].end_state), "not every thread is joined"
                assert all(p == rep or p.lastFork()[1] != rep.lastFork()[1] for p in preds), "joining same thread multiple time"
                assert all(p == rep or len(p.processes & rep.processes) == 0 for p in preds), "thread partition is not respected"
        #TODO check that every thread is eventually joined
        return node_to_trackers
