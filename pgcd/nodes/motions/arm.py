#!/usr/bin/python3

"""
Calibrate arm first!
"""

import steppers
import RPi.GPIO as GPIO
import time

class arm():
    """
    """

    def __init__( self ):
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
        self.stepsTurnTable = 0
        self.stepsCantilever = 0
        self.stepsAnchorpoint = 0
        self.angleTurnTable = 0
        self.angleCantilever = 0
        self.angleAnchorpoint = 0
        self.angleGripper = 0

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

    #17300 steps over all
    def setAngleTurntable( self, angle):
        assert( angle >= 0 and angle <= 270 )
        steps = 17300/270*angle
        delta = steps-self.stepsTurnTable
        self.steps(delta, 0, 0)
        self.stepsTurnTable = steps
        #self.__updateAngleRos__( "angleTurnTable", angle, 270, 10 )

    def getConfigurationMatrixTurntable( self ):
        #angle = sp.rad(270*self.stepsTurnTable/17300)
        angle = self.angleTurnTable
        M = sp.Matrix( [ [sp.cos( angle ), -sp.sin( angle ), 0, 0], [sp.sin(angle), sp.cos(angle), 0, 0], [0, 0, 1, self.offset], [0, 0, 0, 1] ] )
        #print( "caa turn>>", M)
        return M

    #5600 steps over all
    def setAngleCantilever( self, angle):
        assert( angle >= -30 and angle <= 270 )
        steps = 5400/270*angle
        delta = steps-self.stepsCantilever
        self.steps(0, delta, 0)
        self.stepsCantilever = steps
        #self.__updateAngleRos__( "angleCantilever", angle, 270, 10 )

    def getConfigurationMatrixCantilever( self ):
        #angle = sp.rad(270*self.stepsCantilever/5400)
        angle = self.angleCantilever-120
        M = sp.Matrix( [ [1, 0, 0, 0], [0, sp.cos(angle), -sp.sin(angle), 0], [0, sp.sin(angle), sp.cos(angle), 144], [0, 0, 0, 1] ] )
        #print( "caa cant>>", M)
        return M

    def setAngleAnchorPoint( self, angle ):
        assert( angle >= -30 and angle <= 270 )
        steps = 5400/270*angle*5
        delta = steps-self.stepsAnchorpoint
        self.steps(0, 0, delta)
        self.stepsAnchorpoint = steps
        #self.__updateAngleRos__( "angleAnchorpoint", -angle, 300, 10 )

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

    def move( self, turntable, cantilever, anchorpoint ):
        stepsTurnTable = 17300/270*turntable
        stepsCantilever = 5400/270*cantilever
        stepsAnchorpoint = 5400/270*anchorpoint*5
        self.steps(stepsTurnTable, stepsCantilever, stepsAnchorpoint)

    def retractArm( self ):
        self.moveTo(0, 0, 0)
        self.stepsAnchorpoint = 0
        self.stepsTurnTable = 0
        self.stepsCantilever = 0

    def rotateAndGrab( self, angle ):
        self.setAngleCantilever( 30 )
        self.setAngleTurntable( angle )
        self.setAngleCantilever( 0 )

    def idle( self ):
        time.sleep(0.1)

    def wait(self, time):
        time.sleep(time)
    
    def inverse(self, mpName, arg):
        if mpName == "grip":
            assert len(arg) == 1
            middle = (5 + 12.5) / 2
            delta = arg[0] - middle
            return mpName, [middle - delta]
        elif mpName == "move":
            assert len(arg) == 3
            return mpName, [-i for i in args]
        elif mpName == "retractArm" or
             mpName == "setAngleTurntable" or
             mpName == "setAngleCantilever" or
             mpName == "setAngleAnchorPoint" or :
            raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)

if __name__ == "__main__":
    a = arm()
    a.move(90, 210, 150)
    a.move(0, 40, 0)
    a.grip(10.5)
    a.move(0, -70, 0)
    a.grip(5.5)
    a.move(-90, -180, -150)
