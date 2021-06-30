
from motions.proxy import Proxy
from motions.proxy_conf import *

class CarrierProxy(Proxy):

    def __init__( self ):
        super().__init__(carrier_hostname, carrier_username, carrier_password)
        self.stepsCart = 0 # angle in steps
        self.angleCart = 0 # angle in deg
        self.microstepping = 16
        self.x = 0
        self.y = 0


    def __compute_steps__( self, straight, side, rotate ):
        (status, out, err) = self.exec_bloquing("steps", [straight, side, rotate])
        if status == 0:
            (True, 1.0)
        else:
            (False, float(err.read()))


    def setAngleCart( self, angle, x=None, y=None, t=None):
        if x != None:
            assert y != None and t != None
            angle = t
        assert( 0<=angle and angle<=360 )
        steps = (angle/360)*14720*1.6
        (ok, fraction) = self.__compute_steps__( 0,0, steps-self.stepsCart)
        self.stepsCart = self.stepsCart + fraction*(steps-self.stepsCart)
        self.angleCart = self.angleCart + fraction*(angle-self.angleCart)
        if not ok:
            raise RuntimeError(fraction)

    def rotate( self, angle, x=None, y=None, t=None):
        if x != None:
            assert y != None and t != None
            angle = t
        assert( 0<=angle and angle<=360 )
        steps = (angle/360)*14720*1.6
        (ok, fraction) = self.__compute_steps__( 0,0, steps)
        self.stepsCart += self.stepsCart + fraction*(steps-self.stepsCart)
        self.angleCart += self.angleCart + fraction*(angle-self.angleCart)
        if not ok:
            raise RuntimeError(fraction)

    def moveCart( self, distance, x=None, y=None, t=None):
        """
        Distance in mm
        """
        if x != None:
            assert y != None and t != None
            assert t != None
            distance = t
        steps = distance/157.075*3200+(distance/2000)*3200
        (ok, fraction) = self.__compute_steps__( steps, 0, 0 )
        self.x += math.cos(math.radians(self.angleCart))*distance*fraction
        self.y += math.sin(math.radians(self.angleCart))*distance*fraction
        if not ok:
            raise RuntimeError(fraction)

    def strafeCart( self, distance, x=None, y=None, t=None ):
        if x != None:
            assert y != None and t != None
            assert t != None
            distance = t
        steps = distance/157.075*3200+(distance/2000)*3200
        (ok, fraction) = self.__compute_steps__( 0, steps, 0 )
        self.x += math.sin(math.radians(self.angleCart))*distance*fraction
        self.y -= math.cos(math.radians(self.angleCart))*distance*fraction
        if not ok:
            raise RuntimeError(fraction)

    # TODO angle is incremental
    def swipe( self, radius, angle, x=None, y=None, t=None):
        if x != None:
            assert y != None and t != None
            assert t != None
            radius = y
            angle = t
        distanceSide = radius * (angle*2*3.14159/360)
        stepsSide = distanceSide/157.075*3200+(distanceSide/2000)*3200
        stepsAngle = (angle/360)*14720*1.6 * 1.1
        (ok, fraction) = self.__compute_steps__( 0, stepsSide, stepsAngle )
        #update position
        dx = 0 #TODO update the position
        dy = 0 #TODO update the position
        self.x += fraction*dx
        self.y += fraction*dy
        #update angle
        self.stepsCart = self.stepsCart + fraction*(stepsAngle-self.stepsCart)
        self.angleCart = self.angleCart + fraction*(angle-self.angleCart)
        if not ok:
            raise RuntimeError(fraction)

    def idle(self):
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
            else
                assert len(arg) == 4
                if error == None:
                    return mpName, [arg[0], arg[1], arg[2], -arg[3]] #TODO args 0-2
                else
                    fraction = error.args[0]
                    return mpName, [arg[0], arg[1], arg[2], -fraction * arg[3]] #TODO args 0-2
        elif mpName == "setAngleCart":
            if len(arg) == 4:
                return mpName, [arg[0], arg[1], arg[2], arg[2]] #TODO args 0-2
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "swipe":
            if len(arg) == 1:
                if error == None:
                    return mpName, [arg[0], -arg[1]]
                else
                    fraction = error.args[0]
                    return mpName, [arg[0], -fraction * arg[1]]
            else:
                assert len(arg) == 5
                if error == None:
                    return mpName, [arg[0], arg[1], arg[2], arg[3], -arg[4]] #TODO args 0-2
                else
                    fraction = error.args[0]
                    return mpName, [arg[0], arg[1], arg[2], arg[3], -fraction * arg[4]] #TODO args 0-2
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)


    # Cart radius = 215mm, wheel radius = 33.5mm -> 6.41 rev. per wheel per full circle
    # 200 steps per fc * microstepping

