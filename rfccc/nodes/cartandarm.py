#!/usr/bin/env python3.5

"""
Calibrate arm first!
"""

from __future__ import division
import drv8825
import RPi.GPIO as GPIO
import rospy 
from time import sleep, time

import sympy as sp

from multiprocessing import Process



class cart():
    """
    Measurements of the cart:
                     
            _o__      angle: 0/360
           /    \
          /      \
         o\______/o   angle: 60/360
    
    Radius Wheels: 33.5mm
    Radius Cart (Center to wheel bracket): 215mm
    Radius to wheel: 

    """
    def __init__( self ):
        self.motor1 = drv8825.drv8825( pinDir = 11, pinStep = 12, pinEnable = 3, waitingTime=0.002 )
        self.motor2 = drv8825.drv8825( pinDir = 13, pinStep = 15, pinEnable = 3, waitingTime=0.002 )
        self.motor3 = drv8825.drv8825( pinDir = 16, pinStep = 18, pinEnable = 3, waitingTime=0.002 )
        #self.motor4 = drv8825.drv8825( pinDir = 22, pinStep = 7, pinEnable = 3, waitingTime=0.002 )
    

        GPIO.setmode(GPIO.BOARD)
        self.pinEnable = 3
        self.pinMS1 = 5
        self.pinMS2 = 24
        self.pinMS3 = 26
        
        
        GPIO.setup( self.pinEnable, GPIO.OUT )
        GPIO.setup( self.pinMS1, GPIO.OUT )
        GPIO.setup( self.pinMS2, GPIO.OUT )
        GPIO.setup( self.pinMS3, GPIO.OUT )
        
        #self.offset = sp.Matrix( [[1,0,0,63], [0,1,0,0], [0,0,1,0], [0,0,0,1] ] )
        self.offset = 63

        self.stepsCart = 0
        self.angleCart = 0
        self.microstepping = 16 
        self.__microstepping__( 16 )
        #print( self.microstepping )

        self.x = 0
        self.y = 0

        print( "inititalized cart" )


    # Methods concerning moving the cart
    def __motors_start__( self ):
        GPIO.output( self.pinEnable, GPIO.LOW )

    def __motors_shutdown__( self ):
        GPIO.output( self.pinEnable, GPIO.HIGH )

    def __compute_steps__( self, straight, side, rotate ):
        """
        """
        steps_motor1 = ( 0*straight+2*side-rotate )
        steps_motor2 = ( 2*straight+2*side-rotate )
        steps_motor3 = ( -2*straight+2*side-rotate )
       #steps_motor4 = ( 0 ) # *straight+2*side-rotate )
        #print( steps_motor1, steps_motor2, steps_motor3 )
        if steps_motor1 > 0: 
            f = lambda: self.motor1.doStep( steps_motor1, 1 )
        else:
            f = lambda: self.motor1.doStep( -steps_motor1, 0 )

        if steps_motor2 > 0: 
            g = lambda: self.motor2.doStep( steps_motor2, 1 )
        else:
            g = lambda: self.motor2.doStep( -steps_motor2, 0 )
        
        if steps_motor3 > 0: 
            h = lambda: self.motor3.doStep( steps_motor3, 1 )
        else:
            h = lambda: self.motor3.doStep( -steps_motor3, 0 )

        p1 = Process(target = f)
        p2 = Process(target = g)
        p3 = Process(target = h)
        p1.start()
        p2.start()
        p3.start()

        p1.join()
        p2.join()
        p3.join()


    def __setMSPins__( self, MS1, MS2, MS3 ):
        GPIO.output( self.pinMS1, MS1 )
        GPIO.output( self.pinMS2, MS2 )
        GPIO.output( self.pinMS3, MS3 )


    def __microstepping__( self, steps ):
        """
        A4988 config pins
        MS1     MS2     MS3     Microstep Resolution
        Low     Low     Low     Full step
        High    Low     Low     Half step
        Low     High    Low     Quarter step
        High    High    Low     Eighth step
        High    High    High    Sixteenth step
        """
        assert( steps in [1,2,4,8,16] )
        #setStepping = {
        #        1: self.__setMSPins__( 0, 0, 0 ),
        #        2: self.__setMSPins__( 1, 0, 0 ), 
        #        4: self.__setMSPins__( 0, 1, 0 ),
        #        8: self.__setMSPins__( 1, 1, 0 ),
        #        16: self.__setMSPins__( 1, 1, 1 )
        #       }[ steps ]
        GPIO.output( self.pinMS1, GPIO.HIGH )
        GPIO.output( self.pinMS2, GPIO.HIGH )
        GPIO.output( self.pinMS3, GPIO.HIGH )
        
        return steps




    # Cart radius = 215mm, wheel radius = 33.5mm -> 6.41 rev. per wheel per full circle
    # 200 steps per fc * microstepping

    def setAngleCart( self, angle ):
        #self.__motors_start__()
        assert( 0<=angle and angle<=360 )

        steps = (angle/360)*200*6.42*2.15*self.microstepping
        
        #if self.angleCart > steps:
        #    steps = self.angleCart-steps
        #    self.angleCart = self.angleCart-steps
        #else:
        #    steps = steps-self.angleCart
        #    self.angleCart = self.angleCart+steps

        
        #steps = (angle/360)*200*7.4626*1.85

        #print( "angle %d steps" %(steps) )
        if steps > self.angleCart:
            f = lambda: self.motor1.doStep( steps-self.angleCart, 1 )
            g = lambda: self.motor2.doStep( steps-self.angleCart, 1 )
            h = lambda: self.motor3.doStep( steps-self.angleCart, 1 )
        else:
            f = lambda: self.motor1.doStep( self.angleCart-steps, 0 )
            g = lambda: self.motor2.doStep( self.angleCart-steps, 0 )
            h = lambda: self.motor3.doStep( self.angleCart-steps, 0 )

        
        p1 = Process(target = f)
        p2 = Process(target = g)
        p3 = Process(target = h)
        p1.start()
        p2.start()
        p3.start()
    

        now = time()
        future = now+angle/360*41.024


        #print( "now, future, angle ", now, future, angle )
    
        cutoff = now
        while now-cutoff < future-cutoff:
            #print( now-cutoff, future-cutoff, now-cutoff < future-cutoff )
            self.angleCart = sp.rad(sp.N((now-cutoff)/(future-cutoff)*angle))
            now = time()
            #print( "loop", self.angleCart )


        #print( "set angle cart to angle:", self.angleCart, angle )
        p1.join()
        p2.join()
        p3.join()

        

        #self.__compute_steps__( 0,0, steps-self.stepsCart )
        #self.stepsCart = steps
        
        #self.__motors_shutdown__()



    
    def moveCart( self, distance, direction ):
        assert( distance > 0 )
        steps = distance / 210 * 200*self.microstepping
        steps = steps if direction==1 else -steps
        self.__compute_steps__( steps, 0, 0 )

        #steps = (angle/360)*200*6.42*2.15*self.microstepping
        angle = self.angleCart/(200*6.42*self.microstepping)*360
        #(angle/360)*200*6.42*2.15*self.microstepping
        

        dx = sp.cos(angle)*distance
        dy = sp.sin(angle)*distance
        
        if direction == 0:
            self.x = self.x-dx
            self.y = self.y-dy
        else:
            self.x = self.x+dx
            self.y = self.y+dy


    def getConfigurationMatrixCart( self ):
        angle = self.angleCart
        M = sp.Matrix( [ [sp.cos( angle ), -sp.sin( angle ), 0, 0], [sp.sin(angle), sp.cos(angle), 0, 0], [0, 0, 1, self.offset], [0, 0, 0, 1] ] )
        #print( "caa cart>>", M)
        return M



class arm():
    """
    """

    def __init__( self ):
        self.turntable = drv8825.drv8825( pinDir =  38, pinStep = 40, pinEnable = 32, waitingTime=0.002  )
        self.cantilever = drv8825.drv8825( pinDir = 29, pinStep = 31, pinEnable= 32, waitingTime=0.002 )
        self.anchorpoint = drv8825.drv8825( pinDir = 33, pinStep = 35, pinEnable= 32, waitingTime=0.00002 )

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
        now = time()
        future = now+angle/maxAngle*timePerRev

        cutoff = now
        while now-cutoff < future-cutoff:
            #print( now-cutoff, future-cutoff, now-cutoff < future-cutoff )
            setattr( self, angleName, sp.N(sp.rad(sp.N((now-cutoff)/(future-cutoff)*angle)) ) )
            #print( "getAttr", getattr( self, angleName ) )
            #print( "------>", sp.N(sp.rad(sp.N((now-cutoff)/(future-cutoff)*angle)) ))
            rospy.sleep(0.1)

            now = time()
        #setattr( self, angleName, sp.N(sp.rad(angle)))


    #17300 steps over all
    def setAngleTurntable( self, angle ):
        assert( angle >= 0 and angle <= 270 )
        steps = 17300/270*angle
        
        if steps > self.stepsTurnTable:
            self.turntable.doStep( steps-self.stepsTurnTable, 1 )
            self.stepsTurnTable = steps
        else:
            self.turntable.doStep( self.stepsTurnTable-steps, 0 )
            self.stepsTurnTable = steps
        
        self.__updateAngleRos__( "angleTurnTable", angle, 270, 10 )

    def getConfigurationMatrixTurntable( self ):
        #angle = sp.rad(270*self.stepsTurnTable/17300)
        angle = self.angleTurnTable
        M = sp.Matrix( [ [sp.cos( angle ), -sp.sin( angle ), 0, 0], [sp.sin(angle), sp.cos(angle), 0, 0], [0, 0, 1, self.offset], [0, 0, 0, 1] ] )
        #print( "caa turn>>", M)
        return M


    #5600 steps over all
    def setAngleCantilever( self, angle ):
        assert( angle >= -30 and angle <= 270 )

        steps = 5400/270*angle
        
        if steps > self.stepsCantilever:
            self.cantilever.doStep( steps-self.stepsCantilever, 0 )
            self.stepsCantilever = steps
        else:
            self.cantilever.doStep( self.stepsCantilever-steps, 1 )
            self.stepsCantilever = steps
        #print( "set cantilever angle to", steps )

        self.__updateAngleRos__( "angleCantilever", angle, 270, 10 )

    def getConfigurationMatrixCantilever( self ):
        #angle = sp.rad(270*self.stepsCantilever/5400)
        angle = self.angleCantilever-120
        M = sp.Matrix( [ [1, 0, 0, 0], [0, sp.cos(angle), -sp.sin(angle), 0], [0, sp.sin(angle), sp.cos(angle), 144], [0, 0, 0, 1] ] )
        #print( "caa cant>>", M)
        return M



    def setAngleAnchorPoint( self, angle ):
        assert( angle >= -30 and angle <= 270 )

        steps = 5400/270*angle*5
        
        if steps > self.stepsAnchorpoint:
            self.anchorpoint.doStep( steps-self.stepsAnchorpoint, 1 )
            self.stepsAnchorpoint = steps
        else:
            self.anchorpoint.doStep( self.stepsAnchorpoint-steps, 0 )
            self.stepsAnchorpoint = steps
        #
        # print( "set angle to", steps )
        
        self.__updateAngleRos__( "angleAnchorpoint", -angle, 300, 10 )

    def getConfigurationMatrixAnchorPoint( self ):
        #angle = sp.rad(270*self.stepsAnchorpoint/5400)
        angle = self.angleAnchorpoint+120
        M = sp.Matrix( [ [1, 0, 0, 0], [0, sp.cos(angle), -sp.sin(angle), 0], [0, sp.sin(angle), sp.cos(angle), 225], [0, 0, 0, 1] ] )
        #print( "caa ap>>", M)
        return M


    def grip( self, cycle ):
        assert( cycle > 5 and cycle < 12.5 )
        self.p.ChangeDutyCycle( cycle )
        sleep( 1 )
        self.__updateAngleRos__( "angleGripper", -angle, 270, 10 )

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
        f = lambda: self.setAngleCantilever( 0 )
        g = lambda: self.moveToBack() 
        h = lambda: self.setAngleAnchorPoint( 0 )
        p1 = Process(target = f)
        p2 = Process(target = g)
        p3 = Process(target = h)
        p1.start()
        p2.start()
        p3.start()

        p1.join()
        p2.join()
        p3.join()
    
    # Simple Positioning of the arm

    def moveToFront( self ):
        self.setAngleTurntable( 180 )
    
    def moveToLeft( self ):
        self.setAngleTurntable( 270 )
    
    def moveToRight( self ):
        self.setAngleTurntable( 60 )
    
    def moveToBack( self ):
        self.setAngleTurntable( 0 )

if __name__ == "__main__":
    c = cart()
    c.__motors_start__()
    c.setAngleCart( 60 )
    c.__motors_shutdown__()
