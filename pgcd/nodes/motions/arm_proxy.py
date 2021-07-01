
from arm_shared import ArmShared
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

    def steps( self, turntable, cantilever, anchorpoint ):
        self.exec_bloquing("./steps", [turntable, cantilever, anchorpoint])


if __name__ == "__main__":
    c = ArmProxy()
    c.closeGripper()
    time.sleep(1.0)
    c.openGripper()
