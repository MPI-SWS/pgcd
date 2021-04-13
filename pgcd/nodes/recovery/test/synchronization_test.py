from recovery.test.harness import *
from recovery.synchronization import *
import unittest

class SynchronizerTests(unittest.TestCase):

    def test_01(self):
        e = env(1)
        raw_c = '''Recovery =
        def x0 = (P0: Idle()); x1
            x1 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_02(self):
        e = env(1)
        raw_c = '''Recovery =
        def x0 = (P0: Idle()); x1
            x1 = (P0: Idle()); x2
            x2 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_03(self):
        e = env(1)
        raw_c = '''Recovery =
        def x0 = (P0: Wait(1)); x1
            x1 = (P0: Idle()); x2
            x2 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_04(self):
        e = env(1)
        raw_c = '''Recovery =
        def x0 = (P0: Idle()); x1
            x1 = (P0: Wait(1)); x2
            x2 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_05(self):
        e = env(2)
        raw_c = '''Recovery =
        def x0 = (P0: Idle(), P1: Idle()); x1
            x1 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_06(self):
        e = env(2)
        raw_c = '''Recovery =
        def x0 = (P0: Wait(1,2), P1: Idle()); x1
            x1 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_07(self):
        e = env(2)
        raw_c = '''Recovery =
        def x0 = (P0: Idle(), P1: Wait(1)); x1
            x1 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_08(self):
        e = env(2)
        raw_c = '''Recovery =
        def x0 = (P0: Idle(), P1: Idle()); x1
            x1 = (P0: Idle(), P1: Idle()); x2
            x2 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

    def test_09(self):
        e = env(2)
        raw_c = '''Recovery =
        def x0 = (P0: Wait(1), P1: Idle()); x1
            x1 = (P0: Idle(), P1: Idle()); x2
            x2 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)
    
    def test_10(self):
        e = env(2)
        raw_c = '''Recovery =
        def x0 = (P0: Wait(1), P1: Idle()); x1
            x1 = (P0: Idle(), P1: Wait(1)); x2
            x2 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)
    
    def test_11(self):
        e = env(2)
        raw_c = '''Recovery =
        def x0 = l1 || r1
            l1 = (P0: Wait(1)); l2
            r1 = (P1: Wait(2)); r2
            l2 || r2 = x2
            x2 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        #print(c)

#TODO more tests with fork + join

if __name__ == '__main__':
    unittest.main()
