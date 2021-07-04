
import time
from abc import ABC, abstractmethod

class FrankaShared(ABC):

    def __init__( self ):
        # home
        self.a_ref = 0.000000
        self.b_ref = -0.785398
        self.c_ref = -0.000000
        self.d_ref = -2.356195
        self.e_ref = 0.000000
        self.f_ref = 1.570796
        self.g_ref = 0.785398
        # current (assume at home position)
        self.a_cur = self.a_ref
        self.b_cur = self.b_ref
        self.c_cur = self.c_ref
        self.d_cur = self.d_ref
        self.e_cur = self.e_ref
        self.f_cur = self.f_ref
        self.g_cur = self.g_ref

    @abstractmethod
    def run(self, prog):
        return []

    def idle(self):
        time.sleep(0.1)

    def wait(self, t):
        time.sleep(t)

    def homePos(self, aDst = None, bDst = None, cDst = None, dDst = None, eDst = None, fDst = None, gDst = None):
        prog = ["homePos()"]
        self.a_cur = self.a_ref
        self.b_cur = self.b_ref
        self.c_cur = self.c_ref
        self.d_cur = self.d_ref
        self.e_cur = self.e_ref
        self.f_cur = self.f_ref
        self.g_cur = self.g_ref
        self.run(prog)

    def setJoints(self, a, b, c, d, e, f, g, aDst = None, bDst = None, cDst = None, dDst = None, eDst = None, fDst = None, gDst = None):
        if aDst != None:
            assert bDst != None and cDst != None and dDst != None and eDst != None and fDst != None and gDst != None
            a = aDst
            b = bDst
            c = cDst
            d = dDst
            e = eDst
            f = fDst
            g = gDst
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
        if int(self.run(prog)[0]) != 1:
            raise ValueError("grasp")

    def openGripper(self, dist = 0.03):
        prog = ["open()"]
        self.run(prog)

    def moveGripper(self, a, b):
        prog = ["moveGripper("+str(a)+","+str(b)+")"]
        self.run(prog)

    def getPos(self, dist = 0.03):
        prog = ["getPos()"]
        self.run(prog)

    def inverse(self, mpName, arg, error = None):
        if mpName == "homePos":
            if len(arg) == 7:
                return "setJoints", [self.a_ref,self.b_ref,self.c_ref,self.d_ref,self.e_ref,self.f_ref,self.g_ref] + arg
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "addToJoints":
            assert len(arg) == 7
            return mpName, [-a for a in arg]
        elif mpName == "setJoints":
            assert len(arg) == 14
            return mpName, arg[7:14] + arg[0:7]
        elif mpName == "openGripper":
            return "grasp", [0.03] #TODO
        elif mpName == "grasp":
            return "openGripper", []
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
    #f.openGripper()
    #f.homePos()
    f.stop()
