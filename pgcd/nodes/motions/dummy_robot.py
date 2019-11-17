#!/usr/bin/python3

import time
import sympy as sp

# Small example
class DummyRobot():
    
    # local state
    def __init__(self, angle = 0, offset = 0):
        self.angle = angle
        self.offset = offset
    
    # frame shift for mounting point
    def getConfigurationMatrix( self ):
        a = self.angle
        M = sp.Matrix( [ [sp.cos(a),-sp.sin(a), 0, 0],
                         [sp.sin(a), sp.cos(a), 0, 0],
                         [0, 0, 1, self.offset],
                         [0, 0, 0, 1] ] )
        return M
    
    def idle( self ):
        time.sleep(0.2)
    
    def read( self ):
        input('wait for next round ...')
    
    def setAngle( self, angle ):
        delta = (angle - self.angle) / 10
        for i in range(0, 10):
            self.angle += delta
            time.sleep(0.1)
        self.angle = angle
    
    def wait( self, t ):
        time.sleep(t)
