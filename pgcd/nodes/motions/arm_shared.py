
import time
from abc import ABC, abstractmethod

class ArmShared(ABC):

    def __init__( self ):
        self.stepsTurnTable = 0
        self.stepsCantilever = 0
        self.stepsAnchorpoint = 0
        self.angleTurnTable = 0
        self.angleCantilever = 0
        self.angleAnchorpoint = 0
        self.angleGripper = 0
        self.turntableBase = 17300.0/270
        self.cantileverBase = 5400.0/270/4
        self.anchorpointBase = 5400.0/270*5

    @abstractmethod
    def grip( self, cycle ):
        pass

    @abstractmethod
    def steps( self, turntable, cantilever, anchorpoint ):
        pass

    def setAngleTurntable( self, angle, angleDest = None):
        if angleDest != None:
            angle = angleDest
        assert( angle >= 0 and angle <= 270 )
        steps = self.turntableBase*angle
        delta = steps-self.stepsTurnTable
        self.steps(delta, 0, 0)
        self.stepsTurnTable = steps

    def setAngleCantilever( self, angle, angleDest = None):
        if angleDest != None:
            angle = angleDest
        assert( angle >= -30 and angle <= 270 )
        steps = self.cantileverBase*angle
        delta = steps-self.stepsCantilever
        self.steps(0, delta, 0)
        self.stepsCantilever = steps

    def setAngleAnchorPoint( self, angle, angleDest = None):
        if angleDest != None:
            angle = angleDest
        assert( angle >= -30 and angle <= 270 )
        steps = self.anchorpointBase*angle
        delta = steps-self.stepsAnchorpoint
        self.steps(0, 0, delta)
        self.stepsAnchorpoint = steps

    def openGripper(self):
        self.grip(5.5)

    def closeGripper(self):
        #self.grip(12)
        self.grip(10.0)

    def move( self, turntable, cantilever, anchorpoint ):
        stepsTurnTable = self.turntableBase*turntable
        stepsCantilever = self.cantileverBase*cantilever
        stepsAnchorpoint = self.anchorpointBase*anchorpoint
        self.steps(stepsTurnTable, stepsCantilever, stepsAnchorpoint)

    def rotate( self,
                turntable0, cantilever0, anchorpoint0,
                turntable1, cantilever1, anchorpoint1 ):
        self.move(turntable1-turntable0, cantilever1-cantilever0, anchorpoint1-anchorpoint0)

    def moveTo( self, turntable, cantilever, anchorpoint ):
        stepsTurnTable = self.turntableBase*turntable
        stepsCantilever = self.cantileverBase*cantilever
        stepsAnchorpoint = self.anchorpointBase*anchorpoint
        self.steps(self.stepsTurnTable - stepsTurnTable,
                   self.stepsCantilever - stepsCantilever,
                   self.stepsAnchorpoint - stepsAnchorpoint)
        self.stepsAnchorpoint = stepsAnchorpoint
        self.stepsTurnTable = stepsTurnTable
        self.stepsCantilever = stepsCantilever

    def retractArm( self ):
        self.moveTo(0, 0, 0)

    def rotateAndGrab( self, angle ):
        self.setAngleCantilever( 30 )
        self.setAngleTurntable( angle )
        self.setAngleCantilever( 0 )

    def idle( self ):
        time.sleep(0.1)

    def wait(self, t):
        time.sleep(t)

    def inverse(self, mpName, arg, error = None):
        assert error == None #TODO
        if mpName == "grip":
            assert len(arg) == 1
            middle = (5 + 12.5) / 2
            delta = arg[0] - middle
            return mpName, [middle - delta]
        elif mpName == "rotate":
            assert len(arg) == 6
            return mpName, [arg[3], arg[4], arg[5], arg[0], arg[1], arg[2]]
        elif mpName == "move":
            assert len(arg) == 3
            return mpName, [-i for i in args]
        elif mpName == "retractArm" or \
             mpName == "setAngleTurntable" or \
             mpName == "setAngleCantilever" or \
             mpName == "setAngleAnchorPoint" :
            raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)
