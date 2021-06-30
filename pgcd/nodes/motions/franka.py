import franka_robot_arm
import time

class franka():

    def __init__( self, ip = "139.19.176.236"):
        self.f = franka_robot_arm.franke_robot_arm()
        self.f.start(ip)
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

    def stop(self):
        self.f.stop()

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
        self.f.run(prog)

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
        self.f.run(prog)

    def grasp(self, dist = 0.03):
        prog = ["grasp("+str(dist)+")"]
        self.f.run(prog)
        time.sleep(1)

    def open(self, dist = 0.03):
        prog = ["open()"]
        self.f.run(prog)

    def moveGripper(self, a, b):
        prog = ["moveGripper("+str(a)+","+str(b)+")"]
        self.f.run(prog)

    def getPos(self, dist = 0.03):
        prog = ["getPos()"]
        self.f.run(prog)

    def testRun(self):
        prog = ["homePos()",
                "grasp(0.03)",
                "setJointRot(1.5708,0,0,0,0,0,0)",
                "jointMotions()",
                "open()",
                "moveGripper(-0.05,0.1)",
                "getPos()",
                "moveGripper(0.05,0.1)"]
        self.f.run(prog)
        self.stop()

    def inverse(self, mpName, arg, error = None):
        if mpName == "homePos":
            if len(arg) == 7:
                return "setJoints", [0,0,0,0,0,0,0] + arg
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "addToJoints":
            assert len(arg) == 7:
            return mpName, [-a for a in arg]
        elif mpName == "setJoints":
            assert len(arg) == 14:
            return mpName, arg[7:14] + arg[0:7]
        elif mpName == "openGripper":
            return "closeGripper", arg
        elif mpName == "closeGripper":
            return "openGripper", arg
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)

if __name__ == "__main__":
    f = franka()
    f.homePos()
    #f.setJoints( 0.178310,0.635300,-0.449920,-2.122150,2.866786,2.016097,1.141317 )
    #f.setJoints( 0.029303,0.719571,-0.449588,-2.003304,2.698555,2.014047,1.218498 )
    #f.setJoints(0.113141,-0.756021,1.155316,-2.205151,0.138111,2.267583,1.449022)
    #f.setJoints(-0.462060,-1.258402,2.159037,-1.921470,0.191438,2.483492,2.613494)
    #f.grasp(0.02)
    #f.homePos()
    f.getPos()
    #f.open()
    #f.homePos()
    f.stop()
