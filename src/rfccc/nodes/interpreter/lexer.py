import ply.lex as lex
import re


class RoboLexer:
    # RESERVED WORDS
    reserved = (
        'SKIP', 'SEND', 'RECEIVE', 'IF',
        'THEN', 'ELSE', 'WHILE', 'DO',
        'SIN', 'TAN', 'COS', 'ABS', 'SQRT',
        'PRINT'
    )

    tokens = reserved + (
        # LITERALS (identifier, integer, double, string and bool constant)
        'ID', 'MOTION', 'COMPONENT', 'LABEL', 'ICONST', 'DCONST', 'SCONST', 'BCONST',

        # OPERATORS
        'PLUS', 'MINUS', 'TIMES', 'DIVIDE', 'MOD',
        'OR', 'AND', 'NOT', 'LT', 'LE', 'GT', 'GE',
        'EQ', 'NE',

        # ASSIGNMENTS
        'EQUALS',

        # DELIMITERS
        'LPAREN', 'RPAREN',
        'LBRACE', 'RBRACE',
        'COMMA', 'SEMI',
        'COLON', 'DOT'
    )

    # Operators
    t_PLUS = r'\+'
    t_MINUS = r'-'
    t_TIMES = r'\*'
    t_DIVIDE = r'/'
    t_MOD = r'%'
    t_OR = r'\|\|'
    t_AND = r'&&'
    t_NOT = r'!'
    t_LT = r'<'
    t_GT = r'>'
    t_LE = r'<='
    t_GE = r'>='
    t_EQ = r'=='
    t_NE = r'!='

    # Assignment operators
    t_EQUALS = r'='

    # Delimeters
    t_LPAREN = r'\('
    t_RPAREN = r'\)'
    t_LBRACE = r'\{'
    t_RBRACE = r'\}'
    t_COMMA = r','
    t_SEMI = r';'
    t_COLON = r':'
    t_DOT = r'\.'

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
        if re.match(r'm_.*', str(t.value)):
            t.value = str(t.value)[2:]
            t.type = 'MOTION'
        elif re.match(r'id_.*', str(t.value)):
            t.value = str(t.value)[3:]
            t.type = 'COMPONENT'
        elif re.match(r'msg_.*', str(t.value)):
            t.value = str(t.value)[4:]
            t.type = 'LABEL'
        elif re.match(r'(true|false)', str(t.value)):
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
