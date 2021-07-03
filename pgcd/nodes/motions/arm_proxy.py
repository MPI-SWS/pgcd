
from motions.arm_shared import ArmShared
from motions.proxy import Proxy
from motions.proxy_conf import *
import time

class ArmProxy(ArmShared, Proxy):

    def __init__( self ):
        ArmShared.__init__(self)
        Proxy.__init__(self, arm_hostname, arm_username, arm_password)
        self.exec_bloquing("sudo", ["pigpiod"])

    def grip( self, cycle ):
        c = (cycle - 5.5) * 1100 / 6.5 + 700 # from 700 too 1800
        (status, out, err) = self.exec_bloquing("python3", ["grip_pigpio.py", int(c)])
        time.sleep(2.0)

    def steps( self, turntable, cantilever, anchorpoint ):
        self.exec_bloquing("./steps", [turntable, cantilever, anchorpoint])


if __name__ == "__main__":
    c = ArmProxy()
    c.closeGripper()
    c.openGripper()
    #
    #c.setAngleTurntable(0,90)
    #time.sleep(1.0)
    #c.setAngleTurntable(90,0)
    #c.setAngleAnchorPoint(0,110)
    #c.setAngleCantilever(0,240)
    c.rotate(0,0,0,0,270,190)
    c.closeGripper()
    #c.setAngleCantilever(240,0)
    #c.setAngleAnchorPoint(110,0)
    c.rotate(0,270,190,0,0,0)
    c.openGripper()
