import ply.lex as lex
import re


class ChoreographyLexer:
    '''
    Class represents choreography tokenizer (lexer/scanner) converting sequence of characters into sequence of tokens.
    If you are not familiar with generating lexer and parser with python PLY package, go through this tutorial:
    http://www.dabeaz.com/ply/ply.html
    '''

    functions = ('SIN', 'TAN', 'COS', 'ABS', 'SQRT', 'RAD')

    # RESERVED WORDS
    reserved = (
                   'DEF', 'END', 'IN'
               ) + functions

    tokens = reserved + (
        # LITERALS (identifier; motion; component; message type; integer, double, string and bool constant)
        'ID', 'ICONST', 'DCONST', 'BCONST',

        # OPERATORS
        'ARROW', 'PLUS', 'MINUS', 'TIMES', 'DIVIDE', 'MOD', 'POW', 'OR', 'AND', 'NOT', 'LT', 'LE', 'GT', 'GE', 'EQ', 'NE',

        # OTHER
        'EQUALS', 'LPAREN', 'RPAREN', 'LSQUARE', 'RSQUARE', 'LBRACE', 'RBRACE', 'COMMA', 'SEMI', 'COLON', 'AT'
    )

    # Operators
    t_ARROW = r'->'
    t_PLUS = r'\+'
    t_MINUS = r'-'
    t_TIMES = r'\*'
    t_DIVIDE = r'/'
    t_MOD = r'%'
    t_POW = r'\*\*'
    t_OR = r'\|\|'
    t_AND = r'&&'
    t_NOT = r'!'
    t_LT = r'<'
    t_GT = r'>'
    t_LE = r'<='
    t_GE = r'>='
    t_EQ = r'=='
    t_NE = r'!='
    t_EQUALS = r'='
    t_LPAREN = r'\('
    t_RPAREN = r'\)'
    t_LSQUARE = r'\['
    t_RSQUARE = r'\]'
    t_LBRACE = r'\{'
    t_RBRACE = r'\}'
    t_COMMA = r','
    t_SEMI = r';'
    t_COLON = r':'
    t_AT = r'@'

    # Completely ignored characters
    t_ignore = ' \t'

    def __init__(self, **kwargs):
        self.lexer = lex.lex(module=self, **kwargs)
        self.lexer.linestart = 0
        # Identifiers and reserved words
        self.reserved_map = {}
        for r in self.reserved:
            self.reserved_map[r.lower()] = r

    # Newlines
    def t_NEWLINE(self, t):
        r'\n+'
        t.lexer.lineno += t.value.count("\n")

    def t_ID(self, t):
        r'[A-Za-z_][\w_]*'
        if re.match(r'(true|false)', str(t.value)):
            t.value = (t.value == 'true')
            t.type = 'BCONST'
        else:
            t.type = self.reserved_map.get(t.value, "ID")
        return t

    # Floating literal
    def t_DCONST(self, t):
        r'\d+\.\d+'
        t.value = float(t.value)
        return t

    # Integer literal
    def t_ICONST(self, t):
        r'\d+'
        t.value = int(t.value)
        return t

    # String literal
    def t_SCONST(self, t):
        r'\"([^\\\n]|(\\.))*?\"'
        t.value = str(t.value)[1:-1]
        return t

    # Comments
    def t_comment(self, t):
        r'\#.*\n'
        t.lexer.lineno += t.value.count('\n')

    # Errors
    def t_error(self, t):
        print("Illegal character %s" % repr(t.value[0]))
        t.lexer.skip(1)

    # Test it output
    def input(self, data):
        self.lexer.input(data)
        while True:
            tok = self.lexer.token()
            if not tok:
                break
            print(tok)
