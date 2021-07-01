#!/usr/bin/python3

"""
Calibrate arm first!
"""

from arm_shared import ArmShared
import steppers
import RPi.GPIO as GPIO
import time

class arm(ArmShared):
    """
    """

    def __init__( self ):
        ArmShared.__init__(self)
        GPIO.setwarnings(False)
        GPIO.setmode( GPIO.BOARD )
        # 1: turntable
        # 2: cantilever
        # 3: anchorpoint
        self.motors = steppers.Steppers( 3, [38,29,33], [40,31,35], [32,32,32] )
        self.pinServo = 7 #bcm: 4
        GPIO.setup( self.pinServo, GPIO.OUT )
        self.p = GPIO.PWM( self.pinServo, 50 )
        self.p.start( 2.5 )
        #
        self.offset = 63 #height of cart

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

    def getConfigurationMatrixTurntable( self ):
        #angle = sp.rad(270*self.stepsTurnTable/17300)
        angle = self.angleTurnTable
        M = sp.Matrix( [ [sp.cos( angle ), -sp.sin( angle ), 0, 0], [sp.sin(angle), sp.cos(angle), 0, 0], [0, 0, 1, self.offset], [0, 0, 0, 1] ] )
        #print( "caa turn>>", M)
        return M

    def getConfigurationMatrixCantilever( self ):
        #angle = sp.rad(270*self.stepsCantilever/5400)
        angle = self.angleCantilever-120
        M = sp.Matrix( [ [1, 0, 0, 0], [0, sp.cos(angle), -sp.sin(angle), 0], [0, sp.sin(angle), sp.cos(angle), 144], [0, 0, 0, 1] ] )
        #print( "caa cant>>", M)
        return M

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

    def steps( self, turntable, cantilever, anchorpoint ):
        steps = []
        direction = []
        if turntable > 0:
            direction.append( 1 )
        else:
            direction.append( 0 )
        if cantilever > 0:
            direction.append( 0 )
        else:
            direction.append( 1 )
        if anchorpoint > 0:
            direction.append( 1 )
        else:
            direction.append( 0 )
        step_list = list( map( abs, [ turntable, cantilever, anchorpoint] ) )
        time = round( max( list( map( abs, [ turntable/2.0, cantilever/1.0, anchorpoint/2.0 ] ))))
        self.motors.doSteps( time, step_list, direction )

if __name__ == "__main__":
    a = arm()
    a.move(90, 210, 150)
    a.move(0, 40, 0)
    a.grip(10.5)
    a.move(0, -70, 0)
    a.grip(5.5)
    a.move(-90, -180, -150)
