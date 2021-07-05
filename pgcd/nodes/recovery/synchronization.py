import verification.choreography.ast_chor as ast_chor
from verification.choreography.threads import ThreadChecks
from verification.choreography.synchronizability import SyncTracker
from verification.spec.time import DurationSpec
from recovery.motion_annot import *
import sympy as sp
import logging
from copy import copy

log = logging.getLogger("Synchronizer")

class Synchronizer:

    def __init__(self, compensation):
        self.comp = compensation
        self.names = dict()
        # run the thread analysis to get the processes at each state
        tc = ThreadChecks(self.comp, self.comp.world)
        self.state_to_procs = { s:t.processes for (s,t) in tc.perform().items() }
        self.state_to_node = self.comp.mk_state_to_node()
        # extra info to synchronize
        t = SyncTracker([DurationSpec(0,0,False)], { p.name() for p in self.comp.world.allProcesses() })
        t.undef = False
        self.state_to_duration = { self.comp.start_state:t }

    # getting fresh names
    def fresh(self, name):
        if name in self.names:
            i = self.names[name]
            self.names[name] = i+1
            return name + "_" + str(i)
        else:
            self.names[name] = 1
            return name + "_0"

    def last(name):
        if name in self.names:
            return name + "_" + str(self.names[name]-1)
        else:
            return name + "_0"

    def needSync(self, node):
        # a node need to be synced if it is followed by a fork or a motion
        if node.start_state[0] == self.comp.start_state or \
           node.isFork() or \
           node.isMotion() or \
           node.isEnd():
            return True
        else:
            return False

    def modifyLastMotion1(self, state, proc, wait = -1, idle = False):
        assert proc in self.state_to_procs[state]
        node = self.state_to_node[state]
        if node.isMotion():
            for m in node.motions:
                if m.id == proc:
                    if wait > 0:
                        addWait(m, wait)
                    elif idle:
                        addIdle(m)
                    return
            assert False, "?!?"
        else:
            for pred in self.comp.getPred(state):
                for s in pred.start_state:
                    if proc in self.state_to_procs[s]:
                        return self.modifyLastMotion1(s, proc, wait, idle)
            assert False, "!?!"

    # modify the last motion executed by proc before state
    def modifyLastMotion(self, state, proc, wait = -1, idle = False):
        for pred in self.comp.getPred(state):
            for s in pred.start_state:
                if proc in self.state_to_procs[s]:
                    return self.modifyLastMotion1(s, proc, wait, idle)
        assert False, "!?!"

    def getPredDurations(self, node):
        return [ self.state_to_duration[p] for p in node.start_state ]

    def motionDurationForRecovery(self, duration):
        if duration.interruptible:
            duration.max = duration.min
            duration.interruptible = False
        else:
            duration.min = duration.max

    def propagateTime(self, node):
        preds = self.getPredDurations(node)
        if node.isMotion():
            assert len(preds) == 1
            ds = dict()
            for m in node.motions:
                #print("getting " + str(m))
                mo = self.comp.getMotion(m.id, m.mp_name, m.mp_args)
                #print("got " + str(mo))
                d = mo.duration()
                self.motionDurationForRecovery(d) # adapt the duration to recovery
                ds[m.id] = d
            t = preds[0].copy()
            t.motion(ds)
            e = node.end_state[0]
            assert not e in self.state_to_duration
            self.state_to_duration[e] = t
        elif node.isFork():
            assert len(preds) == 1
            for e in node.end_state:
                procs = self.state_to_procs[e]
                t = preds[0].copy()
                t.restrict(procs)
                t.stack.append(DurationSpec(0,0,False))
                assert not e in self.state_to_duration
                self.state_to_duration[e] = t
        elif node.isJoin():
            assert len(preds) > 1
            for t1 in preds:
                for t2 in preds:
                    assert t1.sameStack(t2, 1)
            # FIXME shortcut (not optimal)
            t = preds[0].copy()
            t.stack.pop()
            t.processes = self.state_to_procs[node.end_state[0]]
            for t1 in preds:
                for p in t1.processes:
                    t.duration[p] = t1.stack[-1].concat(t1.duration[p])
            self.state_to_duration[node.end_state[0]] = t
        else:
            assert node.isEnd(), "not expecting ("+str(type(node))+") " + str(node)
            assert len(preds) == 1

    def synchronize(self):
        # insert sync _before_ state
        # traverse like a topological sort
        to_process = copy(self.comp.statements)
        processed = { self.comp.start_state }
        frontier = { self.comp.start_state }
        while len(to_process) > 0:
            assert len(frontier) > 0, str(to_process) + " ? " + str(frontier)
            state = frontier.pop()
            #print("processing", state)
            node = self.state_to_node[state]
            # START bookkeeping about what to process
            to_process.remove(node)
            for s in node.end_state:
                assert not s in processed
                processed.add(s)
                n = self.state_to_node[s]
                if all(pred in processed for pred in n.start_state):
                    frontier.add(s)
            # END bookkeeping
            if self.needSync(node):
                # get the duration for each process in the thread
                procs = self.state_to_procs[state]
                tracker = self.state_to_duration[state]
                log.debug("needSync: " + str(state))
                log.debug(str(tracker))
                # normalise the durations (make all uninteruptible)
                ds2 = dict()
                for p in procs:
                    d = tracker.duration[p]
                    if d.interruptible:
                        ds2[p] = DurationSpec(d.min,d.min,False)
                    else:
                        ds2[p] = d
                # find the best sender: minimize |d.max-d.min| and use name as tie breaker
                # FIXME not the optimal strategy but it will do
                receivers = list(procs)
                receivers.sort(key = lambda x: (ds2[x].max - ds2[x].min, x))
                receivers.reverse()
                sender = receivers.pop()
                # insert messages
                start_state = state
                for r in receivers:
                    end_state = start_state
                    start_state = self.fresh("sync")
                    msg = ast_chor.Message([start_state], sender, r, 'Ok', [], [end_state]) ##XXX
                    self.comp.statements.append(msg)
                    # insert into aux lookup maps
                    self.state_to_node[start_state] = msg
                    self.state_to_procs[start_state] = self.state_to_procs[end_state]
                    self.state_to_duration[start_state] = self.state_to_duration[end_state].copy() # XXX
                if node.start_state[0] == self.comp.start_state:
                    self.comp.start_state = start_state
                else:
                    s = node.start_state[0]
                    for n2 in self.comp.statements:  #find pred and rename
                        if not n2.start_state[0].startswith("sync_") and s in n2.end_state:
                            n2.end_state = [ s2 if s2 != s else start_state for s2 in n2.end_state ]
                # modifyLastMotion if needed
                max_duration = ds2[sender].max
                for r in receivers:
                    max_duration = max(max_duration, ds2[r].max)
                wait = max_duration - ds2[sender].min
                if ds2[sender].min < max_duration and wait > 0:
                    self.modifyLastMotion(state, sender, wait = wait)
                    ds2[sender].min = ds2[sender].min + wait
                    ds2[sender].max = ds2[sender].max + wait
                for r in receivers:
                    if ds2[r].max < max_duration:
                        self.modifyLastMotion(state, r, idle = True)
                # modify the SyncTracker
                tracker.stack[-1] = tracker.stack[-1].concat(ds2[sender])
                tracker.duration[sender] = DurationSpec(0,0,False)
                for r in receivers:
                    tracker.duration[r] = DurationSpec(0,0,False)
            self.propagateTime(node)
        # fill comp.state_to_processes
        tc = ThreadChecks(self.comp, self.comp.world)
        self.comp.state_to_processes = { s:t.processes for (s,t) in tc.perform().items() }
