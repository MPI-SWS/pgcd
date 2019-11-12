#!/usr/bin/python3

import rclpy 
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
        rclpy.sleep(0.1)
    
    def setAngle( self, angle ):
        delta = (angle - self.angle) / 10
        for i in range(0, 10):
            self.angle += delta
            rclpy.sleep(0.1)
        self.angle = angle
    
    def wait( self, time ):
        rclpy.sleep(time)
