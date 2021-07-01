import time
from franka_shared import FrankaShared
from motions.proxy import Proxy
from motions.proxy_conf import *

class FrankaProxy(FrankaShared,Proxy):

    def __init__(self):
        FrankaShared.__init__(self)
        Proxy().__init__(self, franka_hostname, franka_username, franka_password)
        (chan_in, chan_out, chan_err) = self.exec_nonbloquing("./franka_robot_arm", ["139.19.176.51"])
        self.chan_in = chan_in
        self.chan_out = chan_out
        self.chan_err = chan_err

    def __del__(self):
        self.chan_in.close()

    def run(self, prog):
        out = []
        for p in prog:
            self.chan_in.write(p)
            self.chan_in.write('\n')
            self.chan_in.flush()
            out.append(self.chan_out.readline())
        return out


if __name__ == "__main__":
    f = franka()
    f.homePos()
    f.setJoints( 0.605278,-0.752382,-1.742559,-2.683788,-1.266799,2.359626,-2.402420 )
    f.grasp(0.02)
    f.homePos()
    f.setJoints( 2.631401,0.928037,-1.443972,-2.726619,1.378726,2.203010,-2.340407 )
    f.open()
