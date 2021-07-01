from franka_shared import FrankaShared
import franka_robot_arm
import time

class franka(FrankaShared):

    def __init__( self, ip = "139.19.176.236"):
        FrankaShared.__init__(self)
        self.f = franka_robot_arm.franka_robot_arm()
        self.f.start(ip)

    def __del__(self):
        self.f.stop()

    def run(self, prog):
        return self.f.run(prog)

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
