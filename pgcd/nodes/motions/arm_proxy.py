
from motions.proxy import Proxy
from motions.proxy_conf import *
import time

class ArmProxy(Proxy):

    def __init__( self ):
        super().__init__(arm_hostname, arm_username, arm_password)
        self.exec_bloquing("sudo", ["pigpiod"])
        self.stepsTurnTable = 0
        self.stepsCantilever = 0
        self.stepsAnchorpoint = 0
        self.angleTurnTable = 0
        self.angleCantilever = 0
        self.angleAnchorpoint = 0
        self.angleGripper = 0

    def grip( self, cycle ):
        c = (cycle - 5.5) * 1100 / 6.5 + 700 # from 700 too 1800
        (status, out, err) = self.exec_bloquing("python3", ["grip_pigpio.py", int(c)])

    def steps( self, turntable, cantilever, anchorpoint ):
        self.exec_bloquing("./steps", [turntable, cantilever, anchorpoint])

    def setAngleTurntable( self, angle, angleDest = None):
        if angleDest != None:
            angle = angleDest
        assert( angle >= 0 and angle <= 270 )
        steps = 17300/270*angle
        delta = steps-self.stepsTurnTable
        self.steps(delta, 0, 0)
        self.stepsTurnTable = steps

    def setAngleCantilever( self, angle, angleDest = None):
        if angleDest != None:
            angle = angleDest
        assert( angle >= -30 and angle <= 270 )
        steps = 5400/270*angle
        delta = steps-self.stepsCantilever
        self.steps(0, delta, 0)
        self.stepsCantilever = steps

    def setAngleAnchorPoint( self, angle, angleDest = None):
        if angleDest != None:
            angle = angleDest
        assert( angle >= -30 and angle <= 270 )
        steps = 5400/270*angle*5
        delta = steps-self.stepsAnchorpoint
        self.steps(0, 0, delta)
        self.stepsAnchorpoint = steps

    def openGripper(self):
        self.grip(5.5)

    def closeGripper(self):
        self.grip(12)

    def move( self, turntable, cantilever, anchorpoint ):
        stepsTurnTable = 17300/270*turntable
        stepsCantilever = 5400/270*cantilever
        stepsAnchorpoint = 5400/270*anchorpoint*5
        self.steps(stepsTurnTable, stepsCantilever, stepsAnchorpoint)

    def rotate( self,
                turntable0, cantilever0, anchorpoint0,
                turntable1, cantilever1, anchorpoint1 ):
        self.move(turntable1-turntable0, cantilever1-cantilever0, anchorpoint1-anchorpoint0)

    def moveTo( self, turntable, cantilever, anchorpoint ):
        stepsTurnTable = 17300/270*turntable
        stepsCantilever = 5400/270*cantilever
        stepsAnchorpoint = 5400/270*anchorpoint*5
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

    def wait(self, time):
        time.sleep(time)

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


if __name__ == "__main__":
    c = ArmProxy()
    c.closeGripper()
    time.sleep(1.0)
    c.openGripper()
