import time
from motions.proxy import Proxy
from motions.proxy_conf import *

class FrankaProxy():

    def __init__(self):
        super().__init__(franka_hostname, franka_username, franka_password)
        (chan_in, chan_out, chan_err) = self.exec_nonbloquing("./franka_robot_arm", ["139.19.176.51"])
        self.chan_in = chan_in
        self.chan_out = chan_out
        self.chan_err = chan_err
        # home
        self.a_ref = 0.000000
        self.b_ref = -0.785398
        self.c_ref = -0.000000
        self.d_ref = -2.356195
        self.e_ref = 0.000000
        self.f_ref = 1.570796
        self.g_ref = 0.785398
        # current
        self.a_cur = 0.0
        self.b_cur = 0.0
        self.c_cur = 0.0
        self.d_cur = 0.0
        self.e_cur = 0.0
        self.f_cur = 0.0
        self.g_cur = 0.0

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

    def idle(self):
        time.sleep(0.1)

    def wait(self, t):
        time.sleep(t)

    def homePos(self):
        prog = ["homePos()"]
        self.a_cur = self.a_ref
        self.b_cur = self.b_ref
        self.c_cur = self.c_ref
        self.d_cur = self.d_ref
        self.e_cur = self.e_ref
        self.f_cur = self.f_ref
        self.g_cur = self.g_ref
        self.run(prog)

    def setJoints(self, a, b, c, d, e, f, g):
        a1 = a - self.a_cur
        b1 = b - self.b_cur
        c1 = c - self.c_cur
        d1 = d - self.d_cur
        e1 = e - self.e_cur
        f1 = f - self.f_cur
        g1 = g - self.g_cur
        self.addToJoints(a1, b1, c1, d1, e1, f1, g1)

    def addToJoints(self, a,b,c,d,e,f,g):
        prog = ["setJointRot("+str(a)+","+str(b)+","+str(c)+","+str(d)+","+str(e)+","+str(f)+","+str(g)+")",
                "jointMotions()"]
        self.a_cur += a
        self.b_cur += b
        self.c_cur += c
        self.d_cur += d
        self.e_cur += e
        self.f_cur += f
        self.g_cur += g
        self.run(prog)

    def grasp(self, dist = 0.03):
        prog = ["grasp("+str(dist)+")"]
        if int(self.run(prog)[0]) == 0
            raise ValueError("grasp")

    def open(self, dist = 0.03):
        prog = ["open()"]
        self.run(prog)

    def moveGripper(self, a, b):
        prog = ["moveGripper("+str(a)+","+str(b)+")"]
        self.run(prog)

    def getPos(self, dist = 0.03):
        prog = ["getPos()"]
        self.run(prog)
        #TODO print

    def inverse(self, mpName, arg, error = None):
        if mpName == "homePos":
            if len(arg) == 7:
                return "setJoints", [self.a_ref,self.b_ref,self.c_ref,self.d_ref,self.e_ref,self.f_ref,self.g_ref] + arg
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "addToJoints":
            assert len(arg) == 7:
            return mpName, [-a for a in arg]
        elif mpName == "setJoints":
            assert len(arg) == 14:
            return mpName, arg[7:14] + arg[0:7]
        elif mpName == "open":
            return "grasp", [0.03] #TODO
        elif mpName == "grasp":
            return "open", []
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)

if __name__ == "__main__":
    f = franka()
    f.homePos()
    f.setJoints( 0.605278,-0.752382,-1.742559,-2.683788,-1.266799,2.359626,-2.402420 )
    f.grasp(0.02)
    f.homePos()
    f.setJoints( 2.631401,0.928037,-1.443972,-2.726619,1.378726,2.203010,-2.340407 )
    f.open()
