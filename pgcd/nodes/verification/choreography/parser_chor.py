import logging

import ply.yacc as yacc
import collections

import lexer_chor as clexer
from ast_chor import *
from sympy import *
import mpmath
from check_chor import *

from spec.component import Component
from spec.env import Env

import os

class ChoreographyParser:
    precedence = (
        ('left', 'AND', 'OR'),
        ('left', 'LT', 'LE', 'GT', 'GE', 'EQ', 'NE'),
        ('left', 'PLUS', 'MINUS'),
        ('left', 'TIMES', 'DIVIDE', 'MOD'),
        ('left', 'POW'),
        ('right', 'NOT', 'UMINUS', 'SIN', 'TAN', 'COS', 'ABS', 'SQRT')
    )

    def __init__(self, env):
        self.lexer = clexer.ChoreographyLexer()
        self.tokens = self.lexer.tokens
        self.parser = yacc.yacc(module=self)
        if isinstance(env, Component):
            env = Env(env)
        self.env = env

    def parse(self, text, check = True):
        choreography = self.parser.parse(text, self.lexer.lexer)
        choreography.world = self.env.world
        if check:
            self.check_well_formedness(choreography)
        return choreography

    def check_well_formedness(self, chor):
        check = ChoreographyCheck(chor)
        check.check_well_formedness()



    # ------------------------------------- STATEMENTS --------------------------------------

    def p_choreography(self, p):
        ''' choreography : ID EQUALS DEF statements IN LSQUARE expression RSQUARE ID '''
        p[0] = Choreography(p[1], p[4], p[7], p[9])

    def p_statements(self, p):
        ''' statements : statement statements
                       | '''
        if len(p) > 2:
            p[0] = [p[1]] + p[2]
        else:
            p[0] = []

    def p_statement(self, p):
        ''' statement : message
                      | motion
                      | guard
                      | merge
                      | fork
                      | join
                      | checkpoint
                      | end '''
        p[0] = p[1]

    def p_message(self, p):
        'message : ID EQUALS ID ARROW ID COLON ID msgarg SEMI ID'
        p[0] = Message([p[1]], p[3], p[5], p[7], p[8], [p[10]])

    def p_msgarg(self, p):
        ''' msgarg : LPAREN exp RPAREN
                   | LPAREN RPAREN'''
        if len(p) > 3:
            p[0] = p[2]
        else:
            p[0] = []

    def p_exp(self, p):
        ''' exp : expression COMMA exp
                | expression'''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [sympify(p[1])]

    def p_motion(self, p):
        'motion : ID EQUALS LPAREN mspecs RPAREN SEMI ID'
        p[0] = Motion([p[1]], p[4], [p[7]])

    def p_mspecs(self, p):
        ''' mspecs : mspecs COMMA mspecs
                   | ID COLON ID LPAREN funcargs RPAREN '''
        if p[2] == ',':
            p[0] = p[1] + p[3]
        else:
            p[0] = [MotionArg(p[1], p[3], p[5])]

    def p_funcargs(self, p):
        ''' funcargs : expression COMMA funcargs
                     | expression
                     | '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        elif len(p) > 1:
            p[0] = [p[1]]
        else:
            p[0] = []

    def p_guard(self, p):
        ''' guard : ID EQUALS LSQUARE expression RSQUARE ID PLUS gargs '''
        p[0] = GuardedChoice([p[1]], [GuardArg(sympify(p[4]), p[6])] + p[8])

    def p_gargs(self, p):
        ''' gargs : LSQUARE expression RSQUARE ID PLUS gargs
                  | LSQUARE expression RSQUARE ID '''
        if len(p) > 5:
            p[0] = [GuardArg(sympify(p[2]), p[4])] + p[6]
        else:
            p[0] = [GuardArg(sympify(p[2]), p[4])]

    def p_merge(self, p):
        ''' merge : ID PLUS margs EQUALS ID '''
        p[0] = Merge([p[1]] + p[3], [p[5]])

    def p_margs(self, p):
        ''' margs : ID PLUS margs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]

    def p_fork(self, p):
        ''' fork : ID EQUALS fthread OR fargs '''
        fps = [a for a,b in [p[3]] + p[5]]
        ids = [b for a,b in [p[3]] + p[5]]
        assert(fps != [])
        p[0] = Fork([p[1]], fps, ids)

    def p_fthread(self, p):
        ''' fthread : fpOrContract ID
                    | fp ID
                    | ID '''
        if len(p) > 2:
            p[0] = (p[1], p[2])
        else:
            p[0] = (None, p[1])

    def p_fargs(self, p):
        ''' fargs : fthread OR fargs
                  | fthread '''
        if len(p) > 3:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]

    def p_fpOrContract(self, p):
        '''fpOrContract : fp
                        | AT ID LPAREN exp RPAREN '''
        if len(p) > 2:
            p[0] = self.env.getContract(p[2], p[4])
        else:
            p[0] = p[1]

    def p_fp(self, p):
        ''' fp : LBRACE ID ID ID COLON expression RBRACE
               | LBRACE expression RBRACE 
               | LBRACE RBRACE 
               | '''
        if len(p) > 7:
            p[0] = Footprint(Symbol(p[2]), Symbol(p[3]), Symbol(p[4]), sympify(p[6]))
        elif len(p) > 3:
            p[0] = Footprint(Symbol('fpx'), Symbol('fpy'), Symbol('fpz'), sympify(p[2]))
        elif len(p) > 2:
            p[0] = Footprint(Symbol('fpx'), Symbol('fpy'), Symbol('fpz'), sympify(True))
        else:
            p[0] = None

    def p_join(self, p):
        ''' join : ID OR jargs EQUALS ID '''
        p[0] = Join([p[1]] + p[3], [p[5]])

    def p_jargs(self, p):
        ''' jargs : ID OR jargs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]

    def p_end(self, p):
        ''' end : ID EQUALS END '''
        p[0] = End([p[1]])
    
    def p_checkpoint(self, p):
        ''' checkpoint : ID EQUALS CHECKPOINT LPAREN ICONST RPAREN SEMI ID '''
        p[0] = Checkpoint([p[1]],[p[8]],int(p[5]))

    # ------------------------------------EXPRESSIONS--------------------------------------
    # TODO better handling of priority

    def p_expression_ft(self, p):
        '''expression : expression PLUS expression
                  | expression MINUS expression
                  | expression TIMES expression
                  | expression DIVIDE expression
                  | expression MOD expression
                  | expression POW expression
                  | expression AND expression
                  | expression OR expression
                  | expression GT expression
                  | expression GE expression
                  | expression LT expression
                  | expression LE expression
                  | expression EQ expression
                  | expression NE expression'''
        if p[2] == '&&':
            p[0] = And(p[1], p[3])
        elif p[2] == '||':
            p[0] = Or(p[1], p[3])
        elif p[2] == '+':
            p[0] = p[1] + p[3]
        elif p[2] == '-':
            p[0] = p[1] - p[3]
        elif p[2] == '*':
            p[0] = p[1] * p[3]
        elif p[2] == '/':
            p[0] = p[1] / p[3]
        elif p[2] == '**':
            p[0] = p[1] ** p[3]
        elif p[2] == '%':
            p[0] = p[1] % p[3]
        elif p[2] == '>':
            p[0] = p[1] > p[3]
        elif p[2] == '>=':
            p[0] = p[1] >= p[3]
        elif p[2] == '<':
            p[0] = p[1] < p[3]
        elif p[2] == '<=':
            p[0] = p[1] <= p[3]
        elif p[2] == '==':
            p[0] = Eq(p[1], p[3])
        elif p[2] == '!=':
            p[0] = Ne(p[1], p[3])
        else:
            raise Exception("operator not known:" + p[2])

    def p_expression_nd(self, p):
        '''expression : SIN LPAREN expression RPAREN
                      | COS LPAREN expression RPAREN
                      | TAN LPAREN expression RPAREN
                      | ABS LPAREN expression RPAREN
                      | SQRT LPAREN expression RPAREN
                      | RAD LPAREN expression RPAREN
                      | ID LPAREN funcargs RPAREN'''
        # p[1] = symbols('f g h', cls=Function)
        p[0] = str(p[1]) + str(p[2]) + str(p[3]) + str(p[4])
        if p[1] == 'sin':
            p[0] = sin(p[3])
        elif p[1] == 'cos':
            p[0] = cos(p[3])
        elif p[1] == 'tan':
            p[0] = tan(p[3])
        elif p[1] == 'abs':
            p[0] = Abs(p[3])
        elif p[1] == 'sqrt':
            p[0] = sqrt(p[3])
        elif p[1] == 'rad':
            p[0] = mpmath.radians(p[3])
        else:
            p[0] = Function(p[1])(*p[3])

    def p_expression_rd(self, p):
        'expression : LPAREN expression RPAREN'
        p[0] = p[2]

    def p_expression_4th(self, p):
        '''expression : ICONST
                      | DCONST
                      | BCONST'''
        p[0] = sympify(p[1])

    def p_expression_5th(self, p):
        '''expression : ID
                      | NOT expression
                      | MINUS expression %prec UMINUS'''
        if p[1] == u'!':
            p[0] = Not(p[2])
        elif len(p) == 3:
            p[0] = -p[2] #?
        else:
            p[0] = Symbol(p[1])

    # --------------------------------------------ERROR----------------------------

    def p_error(self, p):
        if p:
            print("Line " + str(self.lexer.lexer.lineno) + ". : syntax error at: '{0}' in '{1}'...".format(p.value, p.lexer.lexdata.split('\n')[self.lexer.lexer.lineno-1].strip()))
        else:
            print("Syntax error at EOF")
        raise Exception("Parser encountered an error.")

