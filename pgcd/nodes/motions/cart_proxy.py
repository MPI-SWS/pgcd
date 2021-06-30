from motions.proxy import Proxy
from motions.proxy_conf import *
import math
from time import sleep, time

class CartProxy(Proxy):

    def __init__( self ):
        super().__init__(cart_hostname, cart_username, cart_password)
        self.stepsCart = 0
        self.angleCart = 0
        self.microstepping = 16
        self.x = 0
        self.y = 0

    def __compute_steps__( self, straight, side, rotate ):
        (status, out, err) = self.exec_bloquing("steps", [straight, side, rotate])
        if status == 0:
            return (True, 1.0)
        else:
            return (False, float(err.read()))


    def setAngleCart( self, angle, x=None, y=None, t=None ):
        if x != None:
            assert y != None and t != None
            angle = t
        steps = (angle/360)*200*6.42*6.5*self.microstepping
        self.__compute_steps__( 0,0, steps-self.stepsCart)
        self.stepsCart = steps
        self.angleCart = angle

    def rotate( self, angle, x=None, y=None, t=None ):
        if x != None:
            assert y != None and t != None
            angle = t
        steps = (angle/360)*200*6.42*6.5*self.microstepping
        self.__compute_steps__( 0,0, steps)
        self.stepsCart += steps
        self.angleCart += angle

    def moveCart( self, distance, x=None, y=None, t=None ):
        if x != None:
            assert y != None and t != None
            distance = t
        steps = distance / 410 * 200 * 6 * self.microstepping
        self.__compute_steps__( steps, 0, 0 )
        self.x += math.cos(math.radians(self.angleCart))*distance
        self.y += math.sin(math.radians(self.angleCart))*distance
    
    def strafeCart( self, distance, x=None, y=None, t=None ):
        if x != None:
            assert y != None and t != None
            distance = t
        steps = distance / 410 * 200 * 6 * self.microstepping
        self.__compute_steps__( 0, steps, 0 )
        self.x += math.sin(math.radians(self.angleCart))*distance
        self.y -= math.cos(math.radians(self.angleCart))*distance

    def idle( self ):
        time.sleep(0.1)
    
    def wait(self, time):
        time.sleep(time)

    def inverse(self, mpName, arg, error = None):
        if mpName == "moveCart" or mpName == "strafeCart" or mpName == "rotate":
            if len(arg) == 1:
                if error == None:
                    return mpName, [-arg[0]]
                else
                    fraction = error.args[0]
                    return mpName, [-fraction * arg[0]]
            else:
                assert len(arg) == 4
                if error == None:
                    return mpName, [arg[0], arg[1], arg[2], -arg[3]] #TODO args 0-2
                else:
                    fraction = error.args[0]
                    return mpName, [arg[0], arg[1], arg[2], -fraction * arg[3]] #TODO args 0-2
        elif mpName == "setAngleCart":
            if len(arg) == 4:
                return mpName, [arg[0], arg[1], arg[2], arg[2]] #TODO args 0-2
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)
