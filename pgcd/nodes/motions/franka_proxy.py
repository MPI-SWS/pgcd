import time
from motions.franka_shared import FrankaShared
from motions.proxy import Proxy
from motions.proxy_conf import *

class FrankaProxy(FrankaShared,Proxy):

    def __init__(self):
        FrankaShared.__init__(self)
        Proxy.__init__(self, franka_hostname, franka_username, franka_password)
        (chan_in, chan_out, chan_err) = self.exec_nonbloquing("./frankaLib", [])
        self.chan_in = chan_in
        self.chan_out = chan_out
        self.chan_err = chan_err
        self.run(["connectToRobot("+franka_ip+")"])

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
    f = FrankaProxy()
    f.homePos()
    f.setJoints( -0.828313, 0.116449, -0.099996, -2.796003, -0.055448, 2.902437, -0.088231 )
    f.grasp(0.03)
    f.homePos()
    f.setJoints( 0.960601, 0.101824, -0.042593, -2.81301, 0.031016, 2.885839, 1.677218 )
    f.open()
    f.homePos()
