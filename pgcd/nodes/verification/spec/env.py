from sympy import *

class Env():
    """Overall description of the system, the world, the contracts, etc."""

    def __init__(self, world, contracts = []):
        self.world = world
        self.contracts = dict( ( (c.__name__, c) for c in contracts ) )
    
    def frame(self):
        return self.world.frame();

    def allProcesses(self):
        return self.world.allProcesses();

    def _unpack(self, arg): #TODO args are not vectorized!
        if isinstance(arg, Symbol):
            processes = self.world.allProcesses()
            for p in processes:
                if str(arg) == p.name():
                    return p
            raise Exception("Process not found: " + str(arg))
        else:
            return arg

    def getContract(self, name, args):
        if name in self.contracts:
            # args are sympy expression so process IDs are Symbol
            argsUnpacked = [self._unpack(a) for a in args]
            return self.contracts[name](*argsUnpacked)
        else:
            raise Exception("Contract not found: " + name)
