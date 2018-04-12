import logging

import ply.yacc as yacc

import interpreter.lexer as rlex
from ast_inter import *
from sympy import *

class Parser:

    precedence = (
        ('left', 'AND', 'OR', 'LT', 'LE', 'GT', 'GE', 'EQ', 'NE'),
        ('right', 'NOT'),
        ('left', 'PLUS', 'MINUS'),
        ('left', 'TIMES', 'DIVIDE', 'MOD'),
        ('left', 'POW'),
        ('right', 'UMINUS', 'SIN', 'TAN', 'COS', 'ABS'),
    )

    def __init__(self, **kw):
        logging.basicConfig(
            level=logging.DEBUG,
            filename="parselog.txt",
            filemode="w",
            format="%(filename)10s:%(lineno)4d:%(message)s"
        )
        log = logging.getLogger()
        self.lexer = rlex.Lexer(debuglog=log, debug=True)
        self.tokens = self.lexer.tokens
        self.parser = yacc.yacc(module=self, debuglog=log, debug=True)

    def parse(self, text):
        return Statement(self.parser.parse(text, self.lexer.lexer) + [Exit(S(0))])

    # ------------------------------------- STATEMENT --------------------------------------

    def p_statement_program(self, p):
        ''' statement : statement SEMI statement
                      | receive
                      | send
                      | if
                      | while
                      | assign
                      | motion
                      | print
                      | skip
                      | exit '''
        if len(p) > 2:
            p[0] = p[1] + p[3]
        else:
            p[0] = [p[1]]

    def p_receive_msg(self, p):
        'receive   : RECEIVE LPAREN motion RPAREN  LBRACE actions RBRACE'
        p[0] = Receive(p[3], p[6])

    def p_send_msg(self, p):
        '''send : SEND LPAREN COMPONENT_ID COMMA MSGTYPE COMMA args RPAREN'''
        p[0] = Send(p[3], p[5], p[7])

    def p_if_code(self, p):
        'if : IF LPAREN expression RPAREN LBRACE statement RBRACE ELSE LBRACE statement RBRACE'
        p[0] = If(sympify(p[3]), Statement(p[6]), Statement(p[10]))

    def p_while_code(self, p):
        'while : WHILE LPAREN expression RPAREN LBRACE statement RBRACE'
        p[0] = While(sympify(p[3]), Statement(p[6]))

    def p_assign_code(self, p):
        ''' assign : ID EQUALS expression '''
        p[0] = Assign(id=p[1], value=sympify(p[3]))

    def p_motion_exec(self, p):
        ''' motion : MOTION LPAREN args RPAREN
                   | MOTION '''
        if len(p) > 2:
            p[0] = Motion(p[1], p[3])
        else:
            p[0] = Motion(p[1])

    def p_print_function(self, p):
        '''print : PRINT LPAREN SCONST RPAREN
                 | PRINT LPAREN args RPAREN'''
        p[0]= Print(p[3])

    def p_skip_function(self, p):
        'skip : SKIP'
        p[0] = Skip()
    
    def p_exit_function(self, p):
        'exit : EXIT LPAREN expression RPAREN'
        p[0] = Exit(p[3])

    # ------------------------- ACTIONS, KEY-VALUES AND ARGUMENTS---------------------------

    def p_actions_tuple(self, p):
        ''' actions : actions COMMA actions
                    | LPAREN MSGTYPE COMMA ID COMMA LBRACE statement RBRACE RPAREN '''
        if p[1] == u'(':
            p[0] = [Action(p[2], p[4], Statement(p[7]))]
        else:
            p[0] = p[1] + p[3]

    def p_args_tuple(self, p):
        '''args : expression COMMA args
                | expression'''
        if len(p) > 2:
            p[0] = [sympify(p[1])] + p[3]
        else:
            p[0] = [sympify(p[1])]

    # ------------------------------------EXPRESSIONS--------------------------------------

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
        elif p[2] == '\|\|':
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
                      | ID LPAREN args RPAREN'''
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
        else:
            p[0] = Function(p[1])(*p[3])

    def p_expression_rd(self, p):
        'expression : LPAREN expression RPAREN'
        p[0] = p[2]

    def p_expression_4th(self, p):
        '''expression : ICONST
                      | DCONST
                      | SCONST
                      | BCONST'''
        p[0] = sympify(p[1])

    def p_expression_5th(self, p):
        '''expression : ID
                      | NOT expression
                      | MINUS expression %prec UMINUS'''
        if p[1] == u'!':
            p[0] = Not(p[2])
        elif len(p) == 3:
            p[0] = -p[2]
        else:
            p[0] = Symbol(p[1])

    def p_error(self, p):
        if p:
            print("Line " + str(self.lexer.lexer.lineno) + ". : syntax error at: '{0}' in '{1}'...".format(p.value, p.lexer.lexdata.split('\n')[self.lexer.lexer.lineno-1].strip()))
        else:
            print("Syntax error at EOF")
