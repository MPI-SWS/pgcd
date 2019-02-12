import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime
from array import array
import ctypes

libc = ctypes.CDLL('libc.so.6')

class Steppers():
    def __init__( self, n, pinDir, pinStep, pinNen, ccw=0, usec=500 ):
        self.position = 0
        self.n = n
        self.pinDir = pinDir
        self.pinStep = pinStep
        self.pinNen = pinNen
        self.delta = array('f', [0.0] * n)
        self.usec = usec

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


    def doSteps( self, duration, numberSteps, directions):
        # duration in millisecond
        i = 0
        dt = array('f', [0.0] * self.n)
        #c = array('i', [0] * self.n)
        iteration = duration * 1000 / self.usec
        #print(iteration)
        while i < self.n:
            self.delta[i] = -self.usec
            dt[i] = float(numberSteps[i]) * self.usec / iteration
            #print("dt", i, dt[i])
            if directions[i]:
                GPIO.output( self.pinDir[i], self.PUSH )
            else:
                GPIO.output( self.pinDir[i], self.PULL )
            i += 1

        while iteration > 0:
            iteration -= 1

            # pulse if needed
            i = 0
            while i < self.n:
                if self.delta[i] > 0 :
                    #c[i] += 1
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

            libc.usleep(int(self.usec))
        #i = 0
        #while i < self.n:
        #    print(i,":", c[i])
        #    i+=1
