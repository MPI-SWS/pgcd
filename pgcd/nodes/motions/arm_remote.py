#!/usr/bin/python3

"""
Calibrate arm first!
"""

import steppers
import RPi.GPIO as GPIO
import time
import sys

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

    def grip( self, cycle ):
        assert( cycle > 5 and cycle < 12.5 )
        self.p.ChangeDutyCycle( cycle )
        time.sleep( 2 )
        #self.__updateAngleRos__( "angleGripper", -angle, 270, 10 )

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
    c = arm()
    c.steps( float(sys.argv[1]),  float(sys.argv[2]), float(sys.argv[3]))
