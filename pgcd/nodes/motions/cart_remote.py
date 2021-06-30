#!/usr/bin/python3

import steppers
import RPi.GPIO as GPIO

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
        GPIO.output( self.pinMS1, GPIO.HIGH )
        GPIO.output( self.pinMS2, GPIO.HIGH )
        GPIO.output( self.pinMS3, GPIO.HIGH )

    def __motors_start__( self ):
        GPIO.output( self.pinEnable, GPIO.LOW )

    def __motors_shutdown__( self ):
        GPIO.output( self.pinEnable, GPIO.HIGH )

    def __compute_steps__( self, straight, side, rotate ):
        try:
            self.__motors_start__()
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
        finally:
            self.__motors_shutdown__()



if __name__ == "__main__":
    c = cart()
    c.__compute_steps__(int(sys.argv[1], int(sys.argv[2], int(sys.argv[3])
    return 0
