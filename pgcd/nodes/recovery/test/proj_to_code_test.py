from recovery.test.harness import *
from recovery.synchronization import Synchronizer
from recovery.proj_to_code import *
import unittest

class Proj2CodeTests(unittest.TestCase):

    def test_01(self):
        e = env(1)
        raw_c = '''Recovery =
        def x0 = (P0: Idle()); x1
            x1 = end
        in [ True ] x0'''
        c = parseChoreo(raw_c, e)
        s = Synchronizer(c)
        s.synchronize()
        for proc in e.allProcesses():
            p = Proj2Code(c, proc)
            code = p.getCode()
            #print(code)

#   def test_02(self):
#       e = env(1)
#       raw_c = '''Recovery =
#       def x0 = (P0: Idle()); x1
#           x1 = (P0: Idle()); x2
#           x2 = end
#       in [ True ] x0'''
#       c = parseChoreo(raw_c, e)
#       s = Synchronizer(c)
#       s.synchronize()
#       print(c)
#       for proc in e.allProcesses():
#           print(proc.name())
#           p = Proj2Code(c, proc)
#           code = p.getCode()
#           print(code)

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
        for proc in e.allProcesses():
            #print(proc.name())
            p = Proj2Code(c, proc)
            code = p.getCode()
            #print(code)

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
        for proc in e.allProcesses():
            #print(proc.name())
            p = Proj2Code(c, proc)
            code = p.getCode()
            #print(code)

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
        for proc in e.allProcesses():
            #print(proc.name())
            p = Proj2Code(c, proc)
            code = p.getCode()
            #print(code)

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
        for proc in e.allProcesses():
            #print(proc.name())
            p = Proj2Code(c, proc)
            code = p.getCode()
            #print(code)
