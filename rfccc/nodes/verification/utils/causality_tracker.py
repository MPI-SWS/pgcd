import numpy as np
from copy import copy

class CausalityTracker:

    def __init__(self, process_id_set):
        self.time = 0
        self.process_to_vclock = {}
        self.process_set = copy(process_id_set)
        self.init_vclocks()
        for p in self.process_set:
            self.lastEvent = copy(self.process_to_vclock[p][0])

    def copy(self, tracker):
        self.time = tracker.time
        idx = 0
        for proc in self.process_set:
            self.process_to_vclock[proc] = (np.copy(tracker.process_to_vclock[proc][0]), idx)
            idx += 1

    def init_vclocks(self):
        idx = 0
        for proc in self.process_set:
            self.process_to_vclock[proc] = (np.zeros((len(self.process_set),), dtype=int), idx)
            idx += 1

    def inc_thread_vclock(self, p):
        self.process_to_vclock[p][0][self.process_to_vclock[p][1]] += 1

    def p_concurrent_q(self, p, q):
        return not (self.p_after_q(p, q) or self.p_after_q(q, p))

    def after(self, evtAfter, evtBefore):
        return all( after >= before for after, before in zip(evtAfter, evtBefore))

    def before(self, evtBefore, evtAfter):
        return self.after(evtAfter, evtBefore)

    def p_before_q(self, p, q):
        return self.p_after_q(q, p)

    def p_after_q(self, p, q):
        return self.after(self.process_to_vclock[p][0], self.process_to_vclock[q][0])

    def p_message_q(self, p, q, state):
        #print("msg", p, q)
        #assert self.p_after_q(p, q), 'Process at state "' + ''.join(state) + '": "' + str(self.process_to_vclock[p][0]) + '" isn\'t after process "' + str(self.process_to_vclock[q][0]) + '".'
        self.inc_thread_vclock(p)
        assert self.after(self.process_to_vclock[p][0], self.lastEvent), 'Process at state "' + ''.join(state) + '": "' + str(self.process_to_vclock[p][0]) + '" isn\'t after last event "' + str(self.lastEvent) + '".'

        self.process_to_vclock[q] = (np.copy(self.process_to_vclock[p][0]), self.process_to_vclock[q][1])
        self.lastEvent = self.process_to_vclock[q][0]
        #self.inc_thread_vclock(q)

    def choice_at_p(self, p):
        #print("choice at", p)
        self.inc_thread_vclock(p)
        assert self.after(self.process_to_vclock[p][0], self.lastEvent), 'Process at state "' + ''.join(state) + '": "' + str(self.process_to_vclock[p][0]) + '" isn\'t after last event "' + str(self.lastEvent) + '".'
        self.lastEvent = self.process_to_vclock[p][0]

    def motion(self, duration):
        #print("motion", 1)
        self.time += duration
        self.init_vclocks()
        for p in self.process_set:
            self.lastEvent = copy(self.process_to_vclock[p][0])

    def join_with_causalities(self, causalities, end_states):
        for i in range(len(causalities)):
            assert self.time == causalities[i].time, 'Cannot join states: "' + ' , '.join(end_states) + '" because time doesn\'t match: "' + str(self.time) + ' != ' + str(causalities[i].time) + '".'
            for proc in self.process_set:
                self.process_to_vclock[proc] = (
                np.maximum(self.process_to_vclock[proc][0], causalities[i].process_to_vclock[proc][0]),
                self.process_to_vclock[proc][1])

    def fork_new_thread(self):
        cl = CausalityTracker(self.process_set)
        cl.time = self.time
        idx = 0
        for proc in self.process_set:
            cl.process_to_vclock[proc] = (np.copy(self.process_to_vclock[proc][0]), idx)
            idx += 1
        return cl
