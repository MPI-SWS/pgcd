from spec import *
from compatibility import *
from utils.geometry import *
from cart import *
from arm import *
from franka import *
from refinement import *
from vectorize import *
from mpmath import mp
from experiments_setups import World
from copy import deepcopy
from choreography.projection import Projection
import parser

import unittest


def world()
    w = World(  (0, 0, 0, 0),     # cart
                (1, 0, 0, mp.pi), # carrier
                (1, 0, 0, mp.pi), # franka
                (1, 0, 0, mp.pi), # producer
                (1, 0, 0, mp.pi)) # sensor
    cart = Cart("cart", w, 0)
    arm = Arm("arm", cart)
    carrier = CartSquare("carrier", w, 1)
    franka = FrankaEmikaPanda("franka", w, 2)
    producer = ...("producer", w, 3)
    sensor = ...("senor", w, 4)
    return w
