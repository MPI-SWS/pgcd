#!/usr/bin/python3

import steppers
import RPi.GPIO as GPIO
import sys


class carrier():

    def __init__( self ):
        GPIO.setwarnings(False)
        self.pinInterrupt = 40
        self.motors = steppers.Steppers( 4, [11,13,16,22], [12,15,18,7], [3,3,3,3], pinInterrupt = self.pinInterrupt)

        GPIO.setmode(GPIO.BOARD)
        self.pinEnable = 3
        self.pinMS1 = 5
        self.pinMS2 = 24
        self.pinMS3 = 26

        GPIO.setup( self.pinEnable, GPIO.OUT )
        GPIO.setup( self.pinMS1, GPIO.OUT )
        GPIO.setup( self.pinMS2, GPIO.OUT )
        GPIO.setup( self.pinMS3, GPIO.OUT )
        GPIO.output( self.pinMS1, GPIO.HIGH )
        GPIO.output( self.pinMS2, GPIO.HIGH )
        GPIO.output( self.pinMS3, GPIO.HIGH )

    # Methods concerning moving the cart
    def __motors_start__( self ):
        GPIO.output( self.pinEnable, GPIO.LOW )

    def __motors_shutdown__( self ):
        GPIO.output( self.pinEnable, GPIO.HIGH )

    def __compute_steps__( self, straight, side, rotate ):
        try:
            self.__motors_start__()
            coeff_straight=1
            coeff_side=2
            coeff_rotate=1

            steps_motor1 = coeff_straight * straight + coeff_side * side - coeff_rotate * rotate;
            steps_motor2 = coeff_straight * straight - coeff_side * side - coeff_rotate * rotate;
            steps_motor3 = coeff_straight * straight - coeff_side * side + coeff_rotate * rotate;
            steps_motor4 = coeff_straight * straight + coeff_side * side + coeff_rotate * rotate;

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


            if steps_motor4 > 0:
                direction.append( 1 )
            else:
                direction.append( 0 )

            step_list = list( map( abs, [ steps_motor1, steps_motor2, steps_motor3, steps_motor4] ) )
            max_steps = max( step_list )
            stepspertime = 2.0 # 1.6
            return self.motors.doSteps( round(max_steps/stepspertime), step_list, direction )
        finally:
            self.__motors_shutdown__()

if __name__ == "__main__":
    c = carrier()
    (ok, fraction) = c.__compute_steps__(int(sys.argv[1], int(sys.argv[2], int(sys.argv[3])
    if ok:
        return 0
    else:
        sys.stderr.write(str(fraction))
        return -1
