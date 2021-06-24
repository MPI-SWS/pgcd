#!/usr/bin/python3

import steppers
import RPi.GPIO as GPIO
import sympy as sp
from time import sleep, time
import math

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
        GPIO.setwarnings(False)
        self.motors = steppers.Steppers( 3, [11,13,16], [12,15,18], [3,3,3] )
        GPIO.setmode(GPIO.BOARD)
        self.pinEnable = 3
        self.pinMS1 = 5
        self.pinMS2 = 24
        self.pinMS3 = 26
        #GPIO.setup( self.pinEnable, GPIO.OUT )
        GPIO.setup( self.pinMS1, GPIO.OUT )
        GPIO.setup( self.pinMS2, GPIO.OUT )
        GPIO.setup( self.pinMS3, GPIO.OUT )
        self.offset = 63
        self.stepsCart = 0
        self.angleCart = 0
        self.microstepping = 16
        self.__microstepping__( 16 )
        #print( self.microstepping )
        self.x = 0
        self.y = 0

    def __motors_start__( self ):
        GPIO.output( self.pinEnable, GPIO.LOW )

    def __motors_shutdown__( self ):
        GPIO.output( self.pinEnable, GPIO.HIGH )

    def __compute_steps__( self, straight, side, rotate ):
        """
        """
        steps_motor1 = ( 0*straight+2*side-rotate )
        steps_motor2 = ( 2*straight-1*side-rotate )
        steps_motor3 = ( -2*straight-1*side-rotate )

        steps = []
        direction = []

        if steps_motor1 > 0:
            direction.append( 1 )
        else:
            direction.append( 0 )

        if steps_motor2 > 0:
            direction.append( 1 )
        else:
            direction.append( 0 )

        if steps_motor3 > 0:
            direction.append( 1 )
        else:
            direction.append( 0 )

        step_list = list( map( abs, [ steps_motor1, steps_motor2, steps_motor3] ) )
        max_steps = max( step_list )
        stepspertime = 3.2
        return self.motors.doSteps( round(max_steps/stepspertime), step_list, direction )


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
        assert( steps == 16 )
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
    # factor 4 for the gearbox

    def setAngleCart( self, angle ):
        self.__motors_start__()
        steps = (angle/360)*200*6.42*6.5*self.microstepping
        self.__compute_steps__( 0,0, steps-self.stepsCart)
        self.stepsCart = steps
        self.angleCart = angle
        self.__motors_shutdown__()

    def rotate( self, angle ):
        self.__motors_start__()
        steps = (angle/360)*200*6.42*6.5*self.microstepping
        self.__compute_steps__( 0,0, steps)
        self.stepsCart += steps
        self.angleCart += angle
        self.__motors_shutdown__()

    def moveCart( self, distance ):
        self.__motors_start__()
        steps = distance / 410 * 200 * 6 * self.microstepping
        self.__compute_steps__( steps, 0, 0 )
        self.x += math.cos(math.radians(self.angleCart))*distance
        self.y += math.sin(math.radians(self.angleCart))*distance
        self.__motors_shutdown__()
    
    def strafeCart( self, distance ):
        self.__motors_start__()
        steps = distance / 410 * 200 * 6 * self.microstepping
        self.__compute_steps__( 0, steps, 0 )
        self.x += math.sin(math.radians(self.angleCart))*distance
        self.y -= math.cos(math.radians(self.angleCart))*distance
        self.__motors_shutdown__()

    def idle( self ):
        time.sleep(0.1)
    
    def wait(self, time):
        time.sleep(time)

    def getConfigurationMatrixCart( self ):
        angle = self.angleCart
        M = sp.Matrix( [ [sp.cos( angle ), -sp.sin( angle ), 0, 0], [sp.sin(angle), sp.cos(angle), 0, 0], [0, 0, 1, self.offset], [0, 0, 0, 1] ] )
        #print( "caa cart>>", M)
        return M

    def inverse(self, mpName, arg):
        if mpName == "moveCart" or mpName == "strafeCart" or mpName == "rotate":
            assert len(arg) == 1
            return mpName, [-arg[0]]
        elif mpName == "setAngleCart":
            raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)

if __name__ == "__main__":
    c = cart()
    c.setAngleCart( 45 )
    c.setAngleCart( 0 )
    c.moveCart( 50 )
    c.moveCart(-50 )
