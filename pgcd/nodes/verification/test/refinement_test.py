import unittest
import sys
from verification.spec.env import Env
from verification.spec.component import World
from verification.choreography.projection import *
from verification.choreography.refinement import *
from interpreter.parser import Parser
from cart import CartSquare, Cart
from franka import FrankaEmikaPanda
from crane import Crane
from arm import Arm

def project(fileName, env, process):
    with open(fileName, 'r') as content_file:
        choreoTxt = content_file.read()
    p = Projection()
    choreo = p.parse(choreoTxt, env)
    proj = p.project("test", process)
    return proj

def parse(fileName):
    with open(fileName, 'r') as content_file:
        programTxt = content_file.read()
    p = Parser()
    return p.parse(programTxt)

class RefinementTests(unittest.TestCase):

    def test_01(self):
        w = World()
        carrier = CartSquare("carrier", w, 0)
        crane = Crane("crane", w, 1)
        franka = FrankaEmikaPanda("franka", w, 2)
        env = Env(w, [])
        prefix = "../../../../experiments/recovery_ferry_01/"
        # carrier
        prog = parse(prefix + "carrier.rosl")
        proj = project(prefix + "ferry.choreo", env, carrier)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # crane
        prog = parse(prefix + "crane.rosl")
        proj = project(prefix + "ferry.choreo", env, crane)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # franka
        prog = parse(prefix + "franka.rosl")
        proj = project(prefix + "ferry.choreo", env, franka)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())

    def test_02(self):
        w = World()
        carrier = CartSquare("carrier", w, 0)
        crane = Crane("crane", w, 1)
        franka = FrankaEmikaPanda("franka", w, 2)
        env = Env(w, [])
        prefix = "../../../../experiments/recovery_ferry_02/"
        # carrier
        prog = parse(prefix + "carrier.rosl")
        proj = project(prefix + "ferry.choreo", env, carrier)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # crane
        prog = parse(prefix + "crane.rosl")
        proj = project(prefix + "ferry.choreo", env, crane)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # franka
        prog = parse(prefix + "franka.rosl")
        proj = project(prefix + "ferry.choreo", env, franka)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())

    def test_03(self):
        w = World()
        carrier = CartSquare("carrier", w, 0)
        crane = Crane("crane", w, 1)
        franka = FrankaEmikaPanda("franka", w, 2)
        cart = Cart("cart", w, 3)
        arm = Arm("arm", cart)
        env = Env(w, [])
        prefix = "../../../../experiments/recovery_choice_01/"
        # carrier
        prog = parse(prefix + "carrier.rosl")
        proj = project(prefix + "choice.choreo", env, carrier)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # crane
        prog = parse(prefix + "crane.rosl")
        proj = project(prefix + "choice.choreo", env, crane)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # franka
        prog = parse(prefix + "franka.rosl")
        proj = project(prefix + "choice.choreo", env, franka)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # cart
        prog = parse(prefix + "cart.rosl")
        proj = project(prefix + "choice.choreo", env, cart)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # arm
        prog = parse(prefix + "arm.rosl")
        proj = project(prefix + "choice.choreo", env, arm)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())

    def test_04(self):
        w = World()
        carrier = CartSquare("carrier", w, 0)
        crane = Crane("crane", w, 1)
        franka = FrankaEmikaPanda("franka", w, 2)
        cart = Cart("cart", w, 3)
        arm = Arm("arm", cart)
        env = Env(w, [])
        prefix = "../../../../experiments/recovery_choice_02/"
        # carrier
        prog = parse(prefix + "carrier.rosl")
        proj = project(prefix + "choice.choreo", env, carrier)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # crane
        prog = parse(prefix + "crane.rosl")
        proj = project(prefix + "choice.choreo", env, crane)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # franka
        prog = parse(prefix + "franka.rosl")
        proj = project(prefix + "choice.choreo", env, franka)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # cart
        prog = parse(prefix + "cart.rosl")
        proj = project(prefix + "choice.choreo", env, cart)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())
        # arm
        prog = parse(prefix + "arm.rosl")
        proj = project(prefix + "choice.choreo", env, arm)
        r = Refinement(prog, proj)
        self.assertTrue(r.check())

if __name__ == '__main__':
    unittest.main()
