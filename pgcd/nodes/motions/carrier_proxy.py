
from motions.carrier_shared import CarrierShared
from motions.proxy import Proxy
from motions.proxy_conf import *
import math
import time

class CarrierProxy(CarrierShared, Proxy):

    def __init__( self ):
        CarrierShared.__init__(self)
        Proxy.__init__(self, carrier_hostname, carrier_username, carrier_password)


    def __compute_steps__( self, straight, side, rotate ):
        (status, out, err) = self.exec_bloquing("./steps", [straight, side, rotate])
        if status == 0:
            return (True, 1.0)
        else:
            return (False, float(err.read()))

if __name__ == "__main__":
    c = CarrierProxy()
    c.moveCart(1500)
    c.moveCart(-1500)
