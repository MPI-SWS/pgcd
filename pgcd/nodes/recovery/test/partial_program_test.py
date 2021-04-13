from recovery.partial_program import *
from interpreter.ast_inter import *
from sympy import *
import unittest

class PartialProgTests(unittest.TestCase):

    def test_01(self):
        c = Checkpoint([0])
        p = Statement([Print("1"),
                       c,
                       Print("2")])
        r = resumeAt(p, c)
        assert r.isBlock()
        assert len(r.children) == 2
        assert r.children[0] == c
        assert r.children[1] == p.children[2]

    def test_02(self):
        c = Checkpoint([0])
        p = While(S.true, Statement([Print("1"),c,Print("2")]))
        r = resumeAt(p, c)
        assert r.isBlock()
        assert len(r.children) == 3
        assert r.children[0] == c
        assert r.children[1] == p.program.children[2]
        assert r.children[2] == p

    def test_03(self):
        c = Checkpoint([0])
        spoiler = Checkpoint([0])
        thenCase = Statement([Print("1"),c,Print("2")])
        elseCase = Statement([Print("3"),spoiler,Print("4")])
        p = While(S.true, If(S.true, thenCase, elseCase))
        r = resumeAt(p, c)
        assert r.isBlock()
        assert len(r.children) == 3
        assert r.children[0] == c
        assert r.children[1] == thenCase.children[2]
        assert r.children[2] == p

    def test_03(self):
        c = Checkpoint([0])
        p = Receive(None, Motion("Idle",[]), [Action("1",[],Statement([Skip(),c])), Action("2",[],Skip())])
        r = resumeAt(p, c)
        assert r.isBlock()
        assert len(r.children) == 1
        assert r.children[0] == c
