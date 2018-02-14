import logging

import ply.yacc as yacc

import lexer as rlex
from ast import *


class RoboParser:
    precedence = (
        ('left', 'AND', 'OR', 'LT', 'LE', 'GT', 'GE', 'EQ', 'NE'),
        ('right', 'NOT'),
        ('left', 'PLUS', 'MINUS'),
        ('left', 'TIMES', 'DIVIDE', 'MOD'),
        ('right', 'UMINUS', 'SIN', 'TAN', 'COS', 'ABS')
    )

    # dictionary of variable names
    names = {}

    def __init__(self, **kw):
        logging.basicConfig(
            level=logging.DEBUG,
            filename="parselog.txt",
            filemode="w",
            format="%(filename)10s:%(lineno)4d:%(message)s"
        )
        log = logging.getLogger()
        self.lexer = rlex.RoboLexer(debuglog=log, debug=True)
        self.tokens = self.lexer.tokens
        self.parser = yacc.yacc(module=self, debuglog=log, debug=True)

    def parse(self, text):
        return self.parser.parse(text, self.lexer.lexer)

    def p_statement_skip(self, p):
        'statement : SKIP'
        p[0] = Skip()

    def p_statement_print(self, p):
        'statement : PRINT LPAREN args RPAREN'
        p[0] = UnOp(Type._print, 'print', p[3])

    def p_statement_assign(self, p):
        '''statement : ID COLON EQUALS LBRACE keyval RBRACE
                     | ID DOT ID COLON EQUALS expression
                     | ID COLON EQUALS expression'''
        if p[4] == u'{':
            p[0] = Assign(id=p[1], value=p[5])
        elif p[2] == u'.':
            p[0] = Assign(id=p[1], value=p[6], id_prop=p[3])
        else:
            p[0] = Assign(id=p[1], value=p[4])

    def p_keyval_assign(self, p):
        '''keyval : keyval COMMA keyval
                  | expression COLON expression'''
        if p[2] == u':':
            x = {p[1]: p[3]}
            p[0] = Tuple(tup=x)
        else:
            p[1].merge(p[3])
            p[0] = p[1]

    def p_statement_receive(self, p):
        '''statement : RECEIVE LPAREN MOTION LPAREN args RPAREN RPAREN LBRACE actions RBRACE
                     | RECEIVE LPAREN MOTION RPAREN LBRACE actions RBRACE'''
        if p[4] == u'(':
            p[0] = Receive(Motion(p[3], p[5]), p[9])
        else:
            p[0] = Receive(Motion(p[3]), p[6])

    def p_actions_tuples(self, p):
        '''actions : actions COMMA actions
                   | LPAREN LABEL COMMA ID COMMA LBRACE statement RBRACE RPAREN'''
        if p[1] == u'(':
            p[0] = [Action(p[2], p[4], p[7])]
        else:
            p[0] = p[1] + p[3]

    def p_statement_sequent(self, p):
        'statement : statement SEMI statement'
        p[0] = Statement([p[1], p[3]])

    def p_statement_ifelse(self, p):
        'statement : IF expression THEN LBRACE statement RBRACE ELSE LBRACE statement RBRACE'
        p[0] = If(p[2], p[5], p[9])

    def p_statement_while(self, p):
        'statement : WHILE expression DO LBRACE statement RBRACE'
        p[0] = While(p[2], p[5])

    def p_statement_motion(self, p):
        '''statement : MOTION LPAREN args RPAREN
                     | MOTION'''
        if len(p) > 2:
            p[0] = Motion(p[1], p[3])
        else:
            p[0] = Motion(p[1])

    def p_args_expressions(self, p):
        '''args : args COMMA args
                 | expression'''
        if len(p) > 2:
            p[0] = p[1] + p[3]
        else:
            p[0] = (p[1],)

    def p_statement_send(self, p):
        'statement : SEND LPAREN COMPONENT COMMA LABEL COMMA expression RPAREN'
        p[0] = Send(p[3], p[5], p[7])

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

    def p_expression_uop(self, p):
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
            p[0] = UnOp(Type.id, 'get', p[1])

    def p_error(self, p):
        if p:
            print("Line ", self.lexer.lexer.lineno, ". Syntax error at {0}".format(p.value))
        else:
            print("Syntax error at EOF")
