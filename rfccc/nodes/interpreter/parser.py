import logging

import ply.yacc as yacc

import interpreter.lexer as rlex
from ast import *


class Parser:
    precedence = (
        ('left', 'AND', 'OR', 'LT', 'LE', 'GT', 'GE', 'EQ', 'NE'),
        ('right', 'NOT'),
        ('left', 'PLUS', 'MINUS'),
        ('left', 'TIMES', 'DIVIDE', 'MOD'),
        ('right', 'UMINUS', 'SIN', 'TAN', 'COS', 'ABS')
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
        return self.parser.parse(text, self.lexer.lexer)

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
                      | skip '''
        if len(p) > 2:
            p[0] = Statement([p[1], p[3]])
        else:
            p[0] = p[1]

    def p_receive_msg(self, p):
        'receive   : RECEIVE LPAREN motion RPAREN  LBRACE actions RBRACE'
        p[0] = Receive(p[3], p[6])

    def p_send_msg(self, p):
        'send : SEND LPAREN COMPONENT_ID COMMA MSGTYPE COMMA expression RPAREN'
        p[0] = Send(p[3], p[5], p[7])

    def p_if_code(self, p):
        'if : IF LPAREN expression RPAREN LBRACE statement RBRACE ELSE LBRACE statement RBRACE'
        p[0] = If(p[3], p[6], p[10])

    def p_while_code(self, p):
        'while : WHILE LPAREN expression RPAREN LBRACE statement RBRACE'
        p[0] = While(p[3], p[6])

    def p_assign_code(self, p):
        ''' assign : ID EQUALS LBRACE keyvalue RBRACE
                   | ID DOT ID EQUALS expression
                   | ID EQUALS expression '''
        if p[3] == u'{':
            p[0] = Assign(id=p[1], value=p[4])
        elif p[2] == u'.':
            p[0] = Assign(id=p[1], value=p[5], property=p[3])
        else:
            p[0] = Assign(id=p[1], value=p[3])

    def p_motion_exec(self, p):
        ''' motion : MOTION LPAREN args RPAREN
                   | MOTION '''
        if len(p) > 2:
            p[0] = Motion(p[1], p[3])
        else:
            p[0] = Motion(p[1])

    def p_print_function(self, p):
        'print : PRINT LPAREN args RPAREN'
        p[0] = UnOp(Type._print, 'print', p[3])

    def p_skip_function(self, p):
        'skip : SKIP'
        p[0] = Skip()

    # ------------------------- ACTIONS, KEY-VALUES AND ARGUMENTS---------------------------

    def p_actions_tuple(self, p):
        ''' actions : actions COMMA actions
                    | LPAREN MSGTYPE COMMA ID COMMA LBRACE statement RBRACE RPAREN '''
        if p[1] == u'(':
            p[0] = [Action(p[2], p[4], p[7])]
        else:
            p[0] = p[1] + p[3]

    def p_keyvalue_dictionary(self, p):
        ''' keyvalue : keyvalue COMMA keyvalue
                     | SCONST COLON expression '''
        if p[2] == u':':
            x = {p[1]: p[3]}
            p[0] = Dict(x)
        else:
            p[1].merge(p[3])
            p[0] = p[1]

    def p_args_tuple(self, p):
        '''args : args COMMA args
                | expression'''
        if len(p) > 2:
            p[0] = p[1] + p[3]
        else:
            p[0] = [p[1]]

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
            p[0] = BinOp(Type.dot, '.', p[1], p[3])
        else:
            p[0] = UnOp(Type.id, '', p[1])

    def p_error(self, p):
        if p:
            print("Line " + str(self.lexer.lexer.lineno) + ". : syntax error at: '{0}'...".format(p.value))
        else:
            print("Syntax error at EOF")
