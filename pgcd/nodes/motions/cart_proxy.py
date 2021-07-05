from motions.cart_shared import CartShared
from motions.proxy import Proxy
from motions.proxy_conf import *
import math
import time

class CartProxy(CartShared,Proxy):

    def __init__( self ):
        CartShared.__init__(self)
        Proxy.__init__(self, cart_hostname, cart_username, cart_password)

    def __compute_steps__( self, straight, side, rotate ):
        (status, out, err) = self.exec_bloquing("./steps", [straight, side, rotate])
        if status == 0:
            return (True, 1.0)
        else:
            return (False, float(err.read()))

# 1st motion
#    c.moveCart( 150)
# 2nd motion
#    c.rotate(-7)
#    c.strafeCart( 250)

if __name__ == "__main__":
    c = CartProxy()
    #c.rotate( 10)
    #c.rotate(-10)
    #c.moveCart( 150)
    #c.rotate(-6)
    #c.strafeCart( 250)
    #c.moveCart(-150)
    #c.moveCart( 100)
    #c.strafeCart(-250)
    #c.rotate( 6)
    #c.moveCart(-100)
    c.moveCart(-150)
    c.moveCart( 150)
