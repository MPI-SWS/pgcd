import unittest
import sys
from verification.choreography.threads import *
from verification.choreography.parser_chor import ChoreographyParser

def parse(fileName):
    with open(fileName, 'r') as content_file:
        choreo= content_file.read()
    ChoreographyParser(None).parse(choreo, check = False)

class ParserTests(unittest.TestCase):

    def test_01(self):
        pass

if __name__ == '__main__':
    if len(sys.argv) > 1:
        parse(sys.argv[1])
    else:
        unittest.main()