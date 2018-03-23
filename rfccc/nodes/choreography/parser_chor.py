import logging

import ply.yacc as yacc
import collections

import lexer_chor as clexer
from ast_chor import *
from sympy import *
from graph_chor import *

import os


class ChoreographyParser:
    precedence = (
        ('left', 'AND', 'OR', 'LT', 'LE', 'GT', 'GE', 'EQ', 'NE'),
        ('right', 'NOT'),
        ('left', 'PLUS', 'MINUS'),
        ('left', 'TIMES', 'DIVIDE', 'MOD'),
        ('right', 'UMINUS', 'SIN', 'TAN', 'COS', 'ABS', 'SQRT')
    )

    def __init__(self, **kw):
        logging.basicConfig(
            level=logging.DEBUG,
            filename="chor_parselog.txt",
            filemode="w",
            format="%(filename)10s:%(lineno)4d:%(message)s"
        )
        log = logging.getLogger()
        self.lexer = clexer.ChoreographyLexer(debuglog=log, debug=True)
        self.tokens = self.lexer.tokens
        self.parser = yacc.yacc(module=self, debuglog=log, debug=True)
        self.left_states = set()
        self.right_states = set()
        self.start_state = None
        self.end_appeared = False
        self.state_to_node = {}

    def parse(self, text):
        if len(Choreography.initialized_components) == 0:
            print('---------WARNING: no components initialized, this is only for debugging purposes...-------')
        sequence = self.parser.parse(text, self.lexer.lexer)
        self.well_formdness_check(sequence)
        return sequence

    def well_formdness_check(self, sequence):
        if len(self.left_states ^ self.right_states) != 1:  # start state is always in
            raise Exception('States ' + str((self.left_states ^ self.right_states) - {
                self.start_state}) + ' are not on LHS or RHS!')

        ChoreographyCheck(self.state_to_node, self.start_state)

    def add_to_state_dict(self, lhs_list, rhs_list, node):

        if self.start_state is None:
            self.start_state = lhs_list[0]

        for state in lhs_list:
            if self.left_states.__contains__(state):
                raise Exception('State "' + state + '" appeared on LHS twice...')
            else:
                self.left_states.add(state)
            self.state_to_node[state] = node
        for state in rhs_list:
            if self.right_states.__contains__(state):
                raise Exception('State "' + state + '" appeared on RHS twice...')
            elif state == self.start_state:
                raise Exception('Start state "' + state + '" cannot appear on RHS.')
            else:
                self.right_states.add(state)

    def track_down_states(self, list):
        pass

    # ------------------------------------- STATEMENTS --------------------------------------

    def p_choreography(self, p):
        ''' choreography : ID EQUALS DEF statement IN LSQUARE expression RSQUARE ID '''
        p[0] = Choreography(p[1], p[4], p[7], p[9])

    def p_statement(self, p):
        ''' statement : statement statement
                      | message
                      | motion
                      | guard
                      | merge
                      | fork
                      | join
                      | end '''
        if len(p) > 2:
            p[0] = p[1] + p[2]
        else:
            p[0] = [p[1]]

    def p_message(self, p):
        'message : ID EQUALS ID ARROW ID COLON ID arg SEMI ID'
        p[0] = Message([p[1]], p[3], p[5], p[7], p[8], [p[10]])
        self.add_to_state_dict(p[0].start_state, p[0].end_state, p[0])

    def p_arg(self, p):
        ''' arg : LPAREN exp RPAREN
                | LPAREN RPAREN'''
        if len(p) > 3:
            p[0] = p[2]
        else:
            p[0] = []

    def p_exp(self, p):
        ''' exp : exp COMMA exp
                | expression'''
        if len(p) > 2:
            p[0] = p[1] + p[3]
        else:
            p[0] = [sympify('"' + p[1] + '"')]

    def p_motion(self, p):
        'motion : ID EQUALS LPAREN mspecs RPAREN SEMI ID'
        p[0] = Motion([p[1]], p[4], [p[7]])
        self.add_to_state_dict(p[0].start_state, p[0].end_state, p[0])

    def p_mspecs(self, p):
        ''' mspecs : mspecs COMMA mspecs
                   | ID COLON ID arg'''
        if p[2] == ',':
            p[0] = p[1] + p[3]
        else:
            p[0] = [MotionArg(p[1], p[3], p[4])]

    def p_guard(self, p):
        ''' guard   : ID EQUALS LSQUARE expression RSQUARE ID PLUS gargs '''
        p[0] = Guard([p[1]], [GuardArg(sympify('"' + p[4] + '"'), p[6])] + p[8])
        self.add_to_state_dict(p[0].start_state, p[0].end_state, p[0])

    def p_gargs(self, p):
        ''' gargs : LSQUARE expression RSQUARE ID PLUS gargs
                  | LSQUARE expression RSQUARE ID '''
        if len(p) > 5:
            p[0] = [GuardArg(sympify('"' + p[2] + '"'), p[4])] + p[6]
        else:
            p[0] = [GuardArg(sympify('"' + p[2] + '"'), p[4])]

    def p_merge(self, p):
        ''' merge : ID PLUS margs EQUALS ID '''
        p[0] = Merge([p[1]] + p[3], [p[5]])
        self.add_to_state_dict(p[0].start_state, p[0].end_state, p[0])

    def p_margs(self, p):
        ''' margs : ID PLUS margs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]

    def p_fork(self, p):
        ''' fork : ID EQUALS ID OR fargs '''
        p[0] = Fork([p[1]], [p[3]] + p[5])
        self.add_to_state_dict(p[0].start_state, p[0].end_state, p[0])

    def p_fargs(self, p):
        ''' fargs : ID OR fargs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]

    def p_join(self, p):
        ''' join : ID OR jargs EQUALS ID '''
        p[0] = Join([p[1]] + p[3], [p[5]])
        self.add_to_state_dict(p[0].start_state, p[0].end_state, p[0])

    def p_jargs(self, p):
        ''' jargs : ID OR jargs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]

    def p_end(self, p):
        ''' end : ID EQUALS END '''
        if self.end_appeared:
            raise Exception("Error: 'end' appeared more than once!")
        p[0] = End([p[1]])
        self.add_to_state_dict(p[0].start_state, p[0].end_state, p[0])
        self.end_appeared = True

    # ------------------------------------EXPRESSIONS--------------------------------------

    def p_expression_ft(self, p):
        '''expression : expression PLUS expression
                  | expression MINUS expression
                  | expression TIMES expression
                  | expression DIVIDE expression
                  | expression MOD expression
                  | expression AND expression
                  | expression OR expression
                  | expression GT expression
                  | expression GE expression
                  | expression LT expression
                  | expression LE expression
                  | expression EQ expression
                  | expression NE expression'''
        if p[2] == '&&':
            p[0] = str(p[1]) + ' and ' + str(p[3])
        elif p[2] == '\|\|':
            p[0] = str(p[1]) + ' or ' + str(p[2])
        else:
            p[0] = str(p[1]) + str(p[2]) + str(p[3])

    def p_expression_nd(self, p):
        '''expression : SIN LPAREN expression RPAREN
                      | COS LPAREN expression RPAREN
                      | TAN LPAREN expression RPAREN
                      | ABS LPAREN expression RPAREN
                      | SQRT LPAREN expression RPAREN
                      | ID LPAREN expression RPAREN'''
        p[0] = str(p[1]) + str(p[2]) + str(p[3]) + str(p[4])

    def p_expression_rd(self, p):
        'expression : LPAREN expression RPAREN'
        p[0] = str(p[2])

    def p_expression_4th(self, p):
        '''expression : ICONST
                      | DCONST
                      | SCONST
                      | BCONST'''
        p[0] = str(p[1])

    def p_expression_5th(self, p):
        '''expression : ID
                      | ID DOT ID
                      | NOT expression
                      | MINUS expression %prec UMINUS'''
        if len(p) == 4:
            p[0] = str(p[1]) + '.' + str(p[3])
        elif p[1] == u'!':
            p[0] = ' not ' + str(p[2])
        elif len(p) == 3:
            p[0] = str(p[1]) + str(p[2])
        else:
            p[0] = str(p[1])

    # --------------------------------------------ERROR----------------------------

    def p_error(self, p):
        if p:
            print("Line " + str(self.lexer.lexer.lineno) + ". : syntax error at: '{0}'...".format(p.value))
        else:
            print("Syntax error at EOF")
