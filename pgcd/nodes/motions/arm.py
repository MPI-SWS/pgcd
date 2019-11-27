#!/usr/bin/python3

"""
Calibrate arm first!
"""

import drv8825
import steppers
import RPi.GPIO as GPIO
import time
import sympy as sp
from multiprocessing import Process

# TODO use steppers instead of drv8825

class arm():
    """
    """

    def __init__( self, motionOnSeparateThread = False ):
        self.motionOnSeparateThread = motionOnSeparateThread
        self.turntable = drv8825.drv8825( pinDir =  38, pinStep = 40, pinEnable = 32, waitingTime=0.0005  )
        self.cantilever = drv8825.drv8825( pinDir = 29, pinStep = 31, pinEnable= 32, waitingTime=0.002 )
        self.anchorpoint = drv8825.drv8825( pinDir = 33, pinStep = 35, pinEnable= 32, waitingTime=0.0002 )

        self.pinServo = 7 #bcm: 4

        GPIO.setmode( GPIO.BOARD )
        GPIO.setup( self.pinServo, GPIO.OUT )
        self.p = GPIO.PWM( self.pinServo, 50 )
        self.p.start( 2.5 )

        self.offset = 63 #height of cart

        self.stepsTurnTable = 0
        self.stepsCantilever = 0
        self.stepsAnchorpoint = 0
        
        self.angleTurnTable = 0
        self.angleCantilever = 0
        self.angleAnchorpoint = 0
        self.angleGripper = 0

        print( "inititalized arm" )

    def __updateAngleRos__( self, angleName, angle, maxAngle, timePerRev ):
        """
        Continuously updates the angles s.t. ros can turn the coordinate 
        systems in real time.
        """
        #now = time()
        #future = now+angle/maxAngle*timePerRev
        #cutoff = now
        #while now-cutoff < future-cutoff:
        #    #print( now-cutoff, future-cutoff, now-cutoff < future-cutoff )
        #    setattr( self, angleName, sp.N(sp.rad(sp.N((now-cutoff)/(future-cutoff)*angle)) ) )
        #    #print( "getAttr", getattr( self, angleName ) )
        #    #print( "------>", sp.N(sp.rad(sp.N((now-cutoff)/(future-cutoff)*angle)) ))
        #    rclpy.sleep(0.1)
        #    now = time()
        setattr( self, angleName, sp.N(sp.rad(angle)))

    def _stepTurntable(self, delta):
        if delta >= 0 :
            self.turntable.doStep( delta, 1 )
        else:
            self.turntable.doStep( -delta, 0 )

    #17300 steps over all
    def _setAngleTurntable( self, angle, spawn ):
        assert( angle >= 0 and angle <= 270 )
        steps = 17300/270*angle
        delta = steps-self.stepsTurnTable
        if spawn:
            f = lambda: self._stepTurntable(delta)
            p1 = Process(target = f)
            p1.start()
            p1.join()
        else:
            self._stepTurntable(delta)
        self.stepsTurnTable = steps
        self.__updateAngleRos__( "angleTurnTable", angle, 270, 10 )
    
    def setAngleTurntable( self, angle ):
        self._setAngleTurntable(angle, self.motionOnSeparateThread)

    def getConfigurationMatrixTurntable( self ):
        #angle = sp.rad(270*self.stepsTurnTable/17300)
        angle = self.angleTurnTable
        M = sp.Matrix( [ [sp.cos( angle ), -sp.sin( angle ), 0, 0], [sp.sin(angle), sp.cos(angle), 0, 0], [0, 0, 1, self.offset], [0, 0, 0, 1] ] )
        #print( "caa turn>>", M)
        return M

    def _stepCantilever( self, delta):
        if delta >= 0 :
            self.cantilever.doStep( delta, 0 )
        else:
            self.cantilever.doStep( -delta, 1 )

    #5600 steps over all
    def _setAngleCantilever( self, angle, spawn ):
        assert( angle >= -30 and angle <= 270 )
        steps = 5400/270*angle
        delta = steps-self.stepsCantilever
        if spawn:
            f = lambda: self._stepCantilever(delta)
            p1 = Process(target = f)
            p1.start()
            p1.join()
        else:
            self._stepCantilever(delta)
        self.stepsCantilever = steps
        self.__updateAngleRos__( "angleCantilever", angle, 270, 10 )
    
    def setAngleCantilever( self, angle ):
        self._setAngleCantilever(angle, self.motionOnSeparateThread)

    def getConfigurationMatrixCantilever( self ):
        #angle = sp.rad(270*self.stepsCantilever/5400)
        angle = self.angleCantilever-120
        M = sp.Matrix( [ [1, 0, 0, 0], [0, sp.cos(angle), -sp.sin(angle), 0], [0, sp.sin(angle), sp.cos(angle), 144], [0, 0, 0, 1] ] )
        #print( "caa cant>>", M)
        return M

    def _stepAnchorPoint(self, delta):
        if delta >= 0 :
            self.anchorpoint.doStep( delta, 1 )
        else:
            self.anchorpoint.doStep( -delta, 0 )

    def _setAngleAnchorPoint( self, angle, spawn ):
        assert( angle >= -30 and angle <= 270 )
        steps = 5400/270*angle*5
        delta = steps-self.stepsAnchorpoint
        if spawn:
            f = lambda: self._stepAnchorPoint(delta)
            p1 = Process(target = f)
            p1.start()
            p1.join()
        else:
            self._stepAnchorPoint(delta)
        self.stepsAnchorpoint = steps
        self.__updateAngleRos__( "angleAnchorpoint", -angle, 300, 10 )

    def setAngleAnchorPoint( self, angle ):
        self._setAngleAnchorPoint(angle, self.motionOnSeparateThread)

    def getConfigurationMatrixAnchorPoint( self ):
        #angle = sp.rad(270*self.stepsAnchorpoint/5400)
        angle = self.angleAnchorpoint+120
        M = sp.Matrix( [ [1, 0, 0, 0], [0, sp.cos(angle), -sp.sin(angle), 0], [0, sp.sin(angle), sp.cos(angle), 225], [0, 0, 0, 1] ] )
        #print( "caa ap>>", M)
        return M


    def grip( self, cycle ):
        assert( cycle > 5 and cycle < 12.5 )
        self.p.ChangeDutyCycle( cycle )
        time.sleep( 2 )
        #self.__updateAngleRos__( "angleGripper", -angle, 270, 10 )

    def getConfigurationMatrixGripper( self ):
        #angle = sp.rad(270*self.stepsAnchorpoint/5400)
        #angle = self.angleGripper+120
        angle=0
        M = sp.Matrix( [ [1, 0, 0, 0], [0, sp.cos(angle), -sp.sin(angle), 0], [0, sp.sin(angle), sp.cos(angle), 200], [0, 0, 0, 1] ] )
        #print( "caa grip>>", M)
        return M


    # Motion Primitives

    def rotateAndGrab( self, angle ):
        self.setAngleCantilever( 30 )
        self.setAngleTurntable( angle )
        self.setAngleCantilever( 0 )

    def retractArm( self ):
        f = lambda: self._setAngleCantilever( 0, False )
        g = lambda: self._setAngleTurntable( 0, False ) 
        h = lambda: self._setAngleAnchorPoint( 0, False )
        p1 = Process(target = f)
        p2 = Process(target = g)
        p3 = Process(target = h)
        p1.start()
        p2.start()
        p3.start()
        p1.join()
        p2.join()
        p3.join()
        self.stepsAnchorpoint = 0
        self.stepsTurnTable = 0
        self.stepsCantilever = 0
        self.__updateAngleRos__( "angleTurnTable", 0, 270, 0 )
        self.__updateAngleRos__( "angleAnchorpoint", 0, 300, 0 )
        self.__updateAngleRos__( "angleCantilever", 0, 270, 0 )

    def idle( self ):
        time.sleep(0.1)
    
    # Simple Positioning of the arm

    def moveToFront( self ):
        self.setAngleTurntable( 180 )
    
    def moveToLeft( self ):
        self.setAngleTurntable( 270 )
    
    def moveToRight( self ):
        self.setAngleTurntable( 60 )
    
    def moveToBack( self ):
        self.setAngleTurntable( 0 )
