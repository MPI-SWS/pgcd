from compatibility import *
from utils.geometry import *
from cart import Cart
from static_process import StaticProcess
from refinement import *
from vectorize import *
from experiments_setups import World
from copy import deepcopy
from choreography.projection import Projection
import interpreter.parser as parser
import time
from spec.conf import *

import unittest

# A "microbenchmark" to emphasize the benefit of having the parallel composition as part of the spec.
# Here we that thing can work even where there is some dependencies between the frames.

class XpStackParametricTest(unittest.TestCase):

    def __init__(self):
        pass
