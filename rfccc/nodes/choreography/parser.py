import logging

import ply.yacc as yacc
import collections

import choreography.lexer as clexer
from choreography.ast import *


class ChoreographyParser:

    begin_states = set()

    continue_states = set()

    start_state = None

    end_appeared = False

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

    def parse(self, text):
        tree = self.parser.parse(text, self.lexer.lexer)
        if len(self.begin_states - self.continue_states) != 1: # start state is always in
            raise Exception('States ' + str(self.begin_states ^ self.continue_states) + ' are not on LHS or RHS!')
        return tree

    def ambiguity_check(self, state, is_left):
        if self.start_state is None:
            self.start_state = state
        if is_left:
            if self.begin_states.__contains__(state):
                raise Exception('State "' + state + '" appeared on LHS twice...')
            else:
                self.begin_states.add(state)
        else:
            if self.continue_states.__contains__(state):
                raise Exception('State "' + state + '" appeared on RHS twice...')
            elif state == self.start_state:
                raise Exception('Start state "' + state + '" cannot appear on RHS.')
            else:
                self.continue_states.add(state)

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
            p[0] = Statements([p[1], p[2]])
        else:
            p[0] = p[1]


    def p_message(self, p):
        'message : ID EQUALS ID ARROW ID COLON ID arg SEMI ID'
        p[0] = Message(p[1], p[3], p[5], p[7], p[8], p[10])
        self.ambiguity_check(p[1], True)
        self.ambiguity_check(p[10], False)

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
            p[0] = [p[1]]

    def p_motion(self, p):
        'motion : ID EQUALS LPAREN mspecs RPAREN SEMI ID'
        p[0] = Motion(p[1], p[4], p[7])
        self.ambiguity_check(p[1], True)
        self.ambiguity_check(p[7], False)

    def p_mspecs(self, p):
        ''' mspecs : mspecs COMMA mspecs
                   | ID COLON ID arg'''
        if p[2] == ',':
            p[0] = p[1] + p[3]
        else:
            p[0] = [MotionArg(p[1], p[3], p[4])]

    def p_guard(self, p):
        ''' guard   : ID EQUALS LSQUARE expression RSQUARE ID PLUS gargs '''
        p[0] = Guard(p[1], [GuardArg(p[4], p[6])] + p[8])
        self.ambiguity_check(p[1], True)
        self.ambiguity_check(p[6], False)

    def p_gargs(self, p):
        ''' gargs : LSQUARE expression RSQUARE ID PLUS gargs
                  | LSQUARE expression RSQUARE ID '''
        if len(p) > 5:
            p[0] = [GuardArg(p[2], p[4])] + p[6]
        else:
            p[0] = [GuardArg(p[2], p[4])]
        self.ambiguity_check(p[4], False)

    def p_merge(self, p):
        ''' merge : ID PLUS margs EQUALS ID '''
        p[0] = Merge([p[1]] + p[3], p[5])
        self.ambiguity_check(p[1], True)
        self.ambiguity_check(p[5], False)

    def p_margs(self, p):
        ''' margs : ID PLUS margs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]
        self.ambiguity_check(p[1], True)

    def p_fork(self, p):
        ''' fork : ID EQUALS ID OR fargs '''
        p[0] = Fork(p[1], [p[3]] + p[5])
        self.ambiguity_check(p[1], True)
        self.ambiguity_check(p[3], False)

    def p_fargs(self, p):
        ''' fargs : ID OR fargs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]
        self.ambiguity_check(p[1], False)

    def p_join(self, p):
        ''' join : ID OR jargs EQUALS ID '''
        p[0] = Join([p[1]] + p[3], p[5])
        self.ambiguity_check(p[1], True)
        self.ambiguity_check(p[5], False)

    def p_jargs(self, p):
        ''' jargs : ID OR jargs
                  | ID '''
        if len(p) > 2:
            p[0] = [p[1]] + p[3]
        else:
            p[0] = [p[1]]
        self.ambiguity_check(p[1], True)

    def p_end(self, p):
        ''' end : ID EQUALS END '''
        if self.end_appeared:
            raise Exception("Error: 'end' appeared more than once!")
        p[0] = End(p[1])
        self.ambiguity_check(p[1], True)
        self.end_appeared = True

    # ------------------------------------EXPRESSIONS--------------------------------------

    def p_expression_binop(self, p):
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
        if p[2] == u'+':
            p[0] = BinOp(Type.plus, '+', p[1], p[3])
        elif p[2] == u'-':
            p[0] = BinOp(Type.minus, '-', p[1], p[3])
        elif p[2] == u'*':
            p[0] = BinOp(Type.times, '*', p[1], p[3])
        elif p[2] == u'/':
            p[0] = BinOp(Type.divide, '/', p[1], p[3])
        elif p[2] == u'%':
            p[0] = BinOp(Type.mod, '%', p[1], p[3])
        elif p[2] == u'&&':
            p[0] = BinOp(Type._and, 'and', p[1], p[3])
        elif p[2] == u'||':
            p[0] = BinOp(Type._or, 'or', p[1], p[3])
        elif p[2] == u'>':
            p[0] = BinOp(Type.gt, '>', p[1], p[3])
        elif p[2] == u'>=':
            p[0] = BinOp(Type.ge, '>=', p[1], p[3])
        elif p[2] == u'<':
            p[0] = BinOp(Type.lt, '<', p[1], p[3])
        elif p[2] == u'<=':
            p[0] = BinOp(Type.le, '<=', p[1], p[3])
        elif p[2] == u'==':
            p[0] = BinOp(Type.eq, '==', p[1], p[3])
        elif p[2] == u'!=':
            p[0] = BinOp(Type.ne, '!=', p[1], p[3])

    def p_expression_unop(self, p):
        '''expression : MINUS expression %prec UMINUS
                      | SIN LPAREN expression RPAREN
                      | COS LPAREN expression RPAREN
                      | TAN LPAREN expression RPAREN
                      | ABS LPAREN expression RPAREN
                      | SQRT LPAREN expression RPAREN
                      | NOT expression'''
        if p[1] == u'-':
            p[0] = UnOp(Type.uminus, '-', p[2])
        elif p[1] == u'sin':
            p[0] = UnOp(Type._sin, 'math.sin', p[3])
        elif p[1] == u'cos':
            p[0] = UnOp(Type._cos, 'math.cos', p[3])
        elif p[1] == u'tan':
            p[0] = UnOp(Type._tan, 'math.tan', p[3])
        elif p[1] == u'abs':
            p[0] = UnOp(Type._abs, 'math.fabs', p[3])
        elif p[1] == u'sqrt':
            p[0] = UnOp(Type._sqrt, 'math.sqrt', p[3])
        elif p[1] == u'!':
            p[0] = UnOp(Type._not, 'not', p[2])

    def p_expression_group(self, p):
        'expression : LPAREN expression RPAREN'
        p[0] = Constant(p[2])

    def p_expression_constant(self, p):
        '''expression : ICONST
                      | DCONST
                      | SCONST
                      | BCONST'''
        p[0] = Constant(p[1])

    def p_expression_id(self, p):
        '''expression : ID
                      | ID DOT ID'''
        if len(p) > 2:
            p[0] = Constant(str(p[1]) + '.' + str(p[3]))
        else:
            p[0] = Constant(p[1])

    # --------------------------------------------ERROR----------------------------

    def p_error(self, p):
        if p:
            print("Line " + str(self.lexer.lexer.lineno) + ". : syntax error at: '{0}'...".format(p.value))
        else:
            print("Syntax error at EOF")
