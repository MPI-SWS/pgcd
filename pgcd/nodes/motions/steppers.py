import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime
from array import array
import ctypes

libc = ctypes.CDLL('libc.so.6')

class Steppers():
    def __init__( self, n, pinDir, pinStep, pinNen, ccw=0, usec=500, pinInterrupt = None, pinLimit = None, limitDirection = None):
        """Routines to manage stepper motors with A4988 style drivers.

        Arguments:
        n: number of motors
        pinDir: pin numbers for direction
        pinStep: pin numbers for step
        pinNen: pin numbers for not enable
        ccw: counterclockwise (direction)
        usec: ~period of the loop that procude the pulses
        pinInterrupt: pin number for interruption (normally closed)
        pinLimit: pin number for the limit switches (normally closed)
        limitDirection: is the limit switch at the positive or negative end
        """
        self.position = 0
        self.n = n
        self.pinDir = pinDir
        self.pinStep = pinStep
        self.pinNen = pinNen
        self.delta = array('f', [0.0] * n)
        self.usec = usec
        self.pinInterrupt = pinInterrupt

        if ( ccw ) :
            self.PUSH = GPIO.HIGH
            self.PULL = GPIO.LOW
        else:
            self.PUSH = GPIO.LOW
            self.PULL = GPIO.HIGH

        GPIO.setmode( GPIO.BOARD )
        i = 0
        while i < n:
            GPIO.setup( self.pinDir[i], GPIO.OUT)
            GPIO.setup( self.pinStep[i], GPIO.OUT)
            GPIO.setup( self.pinNen[i], GPIO.OUT)
            i += 1
        if pinInterrupt != None:
            # Normally closed
            GPIO.setup( self.pinInterrupt, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.pinInterrupt, GPIO.RISING)
        if pinLimit != None:
            i = 0
            while i < n:
                GPIO.setup( self.pinLimit[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)
                GPIO.add_event_detect(self.pinLimit[i], GPIO.RISING)

    # toggle the not enable pin
    def on( self ):
        i = 0
        while i < self.n:
            GPIO.output( self.pinNen[i], GPIO.LOW )
            i += 1

    def off( self ):
        i = 0
        while i < self.n:
            GPIO.output( self.pinNen[i], GPIO.HIGH )
            i += 1

    def inOn( self ):
        if GPIO.input( self.pinNen[0] ) == GPIO.LOW:
            return True
        else:
            return False

    def home( self, sequence):
        assert self.pinLimit != None and self.limitDirection != None
        for i in sequence:
            assert i < self.n
            if self.limitDirection[i] > 0:
                GPIO.output( self.pinDir[i], self.PUSH )
            else:
                GPIO.output( self.pinDir[i], self.PULL )
            if not GPIO.input( self.pinLimit[i]):
                while not GPIO.event_detected(self.pinLimit[i]):
                    GPIO.output( self.pinStep[i], GPIO.HIGH )
                    libc.usleep(10)
                    GPIO.output( self.pinStep[i], GPIO.LOW )
                    libc.usleep(int(self.usec) - 10)

    def doSteps( self, duration, numberSteps, directions):
        # duration in millisecond
        i = 0
        dt = array('f', [0.0] * self.n)
        iteration = duration * 1000 / self.usec
        totalIter = iteration
        # compute step period
        while i < self.n:
            self.delta[i] = -self.usec
            dt[i] = float(numberSteps[i]) * self.usec / iteration
            if directions[i]:
                GPIO.output( self.pinDir[i], self.PUSH )
            else:
                GPIO.output( self.pinDir[i], self.PULL )
            i += 1

        while iteration > 0 and (self.pinInterrupt == None or not GPIO.event_detected(self.pinInterrupt)):
            iteration -= 1
            # pulse if needed
            i = 0
            while i < self.n:
                if self.delta[i] > 0 :
                    GPIO.output( self.pinStep[i], GPIO.HIGH )
                    self.delta[i] -= 2 * self.usec
                i += 1
            # update the delta
            i = 0
            while i < self.n:
                self.delta[i] += 2 * dt[i];
                i += 1
            # back to low
            i = 0
            while i < self.n:
                GPIO.output( self.pinStep[i], GPIO.LOW )
                i += 1
            # wait until next iteration
            libc.usleep(int(self.usec))
        # returns a pair with 1st element is "finished normally" (or interrupted) and the 2nd element is how much was done
        return (iteration == 0, (totalIter - iteration) / totalIter)
