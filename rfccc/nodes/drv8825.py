"""
drv8826 driver
                                                                   
2017 marcus@pirron.org                                             
                                                                   
This class implements the control of stepper motors using the      
drv8825 board from pololu:
https://www.pololu.com/product/2133
                                                                   
"""                                                                   

import RPi.GPIO as GPIO
from time import sleep


# Initialize the motor. Expects the pins for direction, step and sleep (w.r.t.
# to the physical numbering on the board). ccw indicates whether the motor is 
# mounted in way which reverses the direction (or uses some kind of gear e.g.).
class drv8825():
    """
    Pin Description:
    STEP:   Make one step
    DIR:    Change direction ( True / False ) of Step - can be omitted
    
    SLEEP:  
    FAULT:  low if H-Bridge FETs are disabled( over current protection, thermal shutdown )
    RESET:
    ENABLE: Enable the driver (default)

    M0, M1, M2: Mode pins for Microstepping

    """
    def __init__( self, pinDir, pinStep, revolutions=200, pinSleep=None, pinM0=None, pinM1=None, pinM2=None, pinReset=None, pinFault=None, pinEnable=None, name='default_drv8825', ccw=0, waitingTime=0.0008 ):
        self.position = 0
        self.name = name
        self.pinDir = pinDir
        self.pinStep = pinStep
        self.pinSleep = pinSleep

        self.pinReset = pinReset
        self.pinFault = pinFault
        self.pinEnable = pinEnable
        
        self.revolutions = revolutions

        self.pinM0 = pinM0
        self.pinM1 = pinM1
        self.pinM2 = pinM2
        
        self.waitingTime = waitingTime
        self.angle=0

        self.microsteps = self.revolutions

        if ( ccw ) :
            self.PUSH = GPIO.HIGH
            self.PULL = GPIO.LOW
        else:
            self.PUSH = GPIO.LOW
            self.PULL = GPIO.HIGH

        GPIO.setmode( GPIO.BOARD )
        GPIO.setup( self.pinDir, GPIO.OUT)
        GPIO.setup( self.pinStep, GPIO.OUT)
        #GPIO.setup( self.pinSleep, GPIO.OUT)

        
    def __repr__(self):
        return '%s> angle %s, steps: %s, %s' %( self.name, self.angle, self.position, self.position)
 
# Methods  Microstepping

    def __toggle_M_pins__( self, listOfStates ):
        GPIO.output ( self.pinM0, listOfStates[0] )
        GPIO.output ( self.pinM1, listOfStates[1] )
        GPIO.output ( self.pinM2, listOfStates[2] )


    def setMicroStepResolution( resolution ):
        """
        Set the resolution of the Microstepping.

        MODE0 	MODE1 	MODE2 	Microstep Resolution  Example: 200 steps per circle
        Low 	Low 	Low 	Full step               200
        High 	Low 	Low 	Half step               400
        Low 	High 	Low 	1/4 step                800
        High 	High 	Low 	1/8 step                1600
        Low 	Low 	High 	1/16 step               3200
        High 	Low 	High 	1/32 step               self.microsteps
        Low 	High 	High 	1/32 step               self.microsteps
        High 	High 	High 	1/32 step               self.microsteps
        """
        assert( resolution in [ 1, 2, 4, 8, 16, 32] )
        configuration = { 1: [0,0,0],
                2: [1,0,0],
                4: [0,1,0],
                8: [1,1,0],
                16: [0,0,1],
                32: [1,1,1]
                }
        self.__toggle_M_pins__( configuration[ resolution ] )
        self.microsteps = self.revolutions*resolution


    def setMicroSteps( steps ):
        """Sets the number of microsteps per revolution"""
        self.setMicroStepResolution( steps/self.revolutions )


    def disable( self ):
        """Disable Motor (set SLEEP to low)"""
        return GPIO.output( self.pinEnable, GPIO.LOW )    
    
    def enable( self ):
        """Enable Motor (set SLEEP to high)"""
        return GPIO.output( self.pinEnable, GPIO.HIGH )
    
    def isEnabled( self ):
        """Return status: 0 if disabled, 1 if enabled""" 
        if GPIO.input( self.pinSleep ) == GPIO.HIGH:
            return False
        else:
            return True    


    def doStep( self, numberSteps, direction=1 ):
        """Rotate the motor a number of steps"""
        if direction:
            GPIO.output( self.pinDir, self.PUSH )
        else:
            GPIO.output( self.pinDir, self.PULL )
        
        for i in range( 0, int( numberSteps ) ):
            GPIO.output( self.pinStep, GPIO.HIGH )        
            GPIO.output( self.pinStep, GPIO.LOW )
            sleep( self.waitingTime )
            if direction:
                self.position += 1
                self.angle = self.angle + 6.2831/self.microsteps
            else:    
                self.position -= 1
                self.angle = self.angle - 6.2831/self.microsteps

    def doStepRight( self, numberSteps ):
        self.doStep( int( numberSteps ), 1 )

    def doStepLeft( self, numberSteps ):
        self.doStep( int( numberSteps ), 0 )

    def getAngle( self ):
        return self.angle
    
    def rotate( self, angle ):
        """ 
        Rotate motor to a specific angle.
        Angle is given in rad [0, 2pi)
        """
        if angle > self.angle:
            angle = angle-self.angle
            self.doStep( angle*self.microsteps/6.2831,1 )
            self.angle = angle
        else:
            angle = self.angle-angle
            self.doStep( angle*self.microsteps/6.2381,0 )
            self.angle = angle




def main():
    #s = drv8825( 18, 16, 32, waitingTime=0.0004 )
    s = drv8825( pinDir=40, pinStep=38, pinSleep=16, waitingTime=0.0004 )
    GPIO.setup ( 26, GPIO.OUT )
    GPIO.output( 26, GPIO.HIGH )
    print( s )
    s.rotate( 6.28319 )
    print( s )
    s.rotate( 6.28319/4 )
    print( s )
    s.doStep( 64000 )
    print( s )


if __name__ == "__main__":
    main()


