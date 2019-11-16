import franka_robot_arm
import time

class franka():
    
    def __init__( self, ip = "139.19.176.236"):
        self.ip = ip
        self.f = franka_robot_arm.franke_robot_arm()

    def idle( self ):
        time.sleep(0.1)
    
    def homePos(self):
        prog = ["connectToRobot("+ip+")", "homePos()"]
        self.f.run(prog)

    def setJoint(self, a,b,c,d,e,f,g):
        prog = ["connectToRobot("+ip+")",
                "setJointRot("+str(a)+","+str(b)+","+str(c)+","+str(d)+","+str(e)+","+str(f)+","+str(g)+")"
                "jointMotions()"]
        self.f.run(prog)

    def grasp(self, dist = 0.03):
        prog = ["connectToRobot("+ip+")", "grasp("+str(dist)+")"]
        self.f.run(prog)
    
    def open(self, dist = 0.03):
        prog = ["connectToRobot("+ip+")", "open()"]
        self.f.run(prog)

    def moveGripper(self, a, b):
        prog = ["connectToRobot("+ip+")", "moveGripper("+str(a)+","+str(b)+")"]
        self.f.run(prog)
