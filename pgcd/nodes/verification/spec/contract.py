from sympy import *
from abc import ABC, abstractmethod
from spec.time import *

class AssumeGuaranteeContract(ABC):

    # Part about the processes for this contract

    @abstractmethod
    def components(self):
        """The component part of this contract"""
        pass

    def inputs(self):
        """external inputs"""
        return [ v for c in self.components() for v in c.inputVariables() ]
    
    def outputs(self):
        """outputs"""
        return [ v for c in self.components() for v in c.outputVariables() ]

    # Part about the pre/post/inv

    def duration(self):
        return DurationSpec(1, 1, False)

    def preA(self):
        """assumption over the contract inputs at the beginning"""
        return S.true

    def preG(self):
        """guarantees over the contract outputs at the beginning"""
        return S.false

    def preFP(self, point):
        """footprint at the start"""
        return S.true

    def invA(self):
        """assumption during the execution of the contract (as function of time)"""
        return S.true

    def invG(self):
        """guarantees during the execution of the contract (as function of time)"""
        return S.true

    def invFP(self, point):
        """footprint of the invariant"""
        return S.true

    def postA(self):
        """assumption when the contract terminates"""
        return S.true

    def postG(self):
        """guarantees when the contract terminates"""
        return S.true

    def postFP(self, point):
        """footprint of the postcondition"""
        return S.true

    # manipulating contracts

    def compose(self, contract, connection):
        """returns a new contract which is the composition of two contracts"""
        pass #TODO

    def checkCompatibility(self, contract, connection):
        """returns VCs to check that two contracts are compatible"""
        pass #TODO
