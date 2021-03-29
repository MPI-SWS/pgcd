from verification.test.static_process import PointProcess
from verification.test.experiments_setups import World
from verification.spec.env import Env
from choreography.parser_chor import ChoreographyParser

def env(n):
    w = World()
    procs = [ PointProcess('P'+str(i),i,0,0,w,i) for i in range(0,n) ]
    return Env(w, [])

def parseChoreo(chor, env):
    return ChoreographyParser(env).parse(chor, check = False)

# comp info
# expected result
