from sympy import *

def timeSymbol():
    return Symbol("t")

def timifyVar(var):
    return Function(var.name)(timeSymbol())
    
def timifyFormula(var, pred):
    time = { v: timifyVar(v) for v in var }
    return pred.subs(time)
    
def deTimifyFormula(var, pred):
    detime = { timifyVar(v): v for v in var }
    res = pred.subs(detime)
    # print("deTimifyFormula")
    # print("detime", detime)
    # print("res", res)
    return res

# Specification for the timing aspect of a motion primitive
class DurationSpec():

    def __init__(self, _min = 0, _max = float('inf'), interruptible = True):
        self.min = _min
        self.max = _max
        self.interruptible = interruptible

    def __str__(self):
        return "DurationSpec(" + str(self.min) + ", " + str(self.max) + ", " + str(self.interruptible) + ")"

    def __eq__(self, other):
        if isinstance(other, DurationSpec):
            return self.min == other.min and \
                   self.max == other.max and \
                   self.interruptible == other.interruptible
        else:
            return False

    def copy(self):
        return DurationSpec(self.min, self.max, self.interruptible)

    def valid(self):
        return self.min <= self.max

    def fixed(self):
        return self.min == self.max

    def concat(self, ds):
        assert not self.interruptible, "cannot add something after an non interruptible motion: " + str(self) + " then " + str(ds)
        if ds.interruptible:
            new_min = self.max + ds.min
            new_max = self.min + ds.max
            assert(new_min <= new_max)
            return DurationSpec(new_min, new_max, ds.interruptible)
        else:
            return DurationSpec(self.min + ds.min, self.max + ds.max, ds.interruptible)

    def consume(self, ds):
        """inverse from concat, remove a duration from this one"""
        assert not ds.interruptible, "cannot consume from an interruptible motion: " + str(self) + " - " + str(ds)
        assert(ds.max <= self.min)
        new_min = self.min - ds.max
        new_max = self.max - ds.min
        return DurationSpec(new_min, new_max, self.interruptible)

    def intersect(self, ds):
        assert(self.interruptible or ds.interruptible or (self.fixed() and ds.fixed())), "cannot intersect " + str(self) + " and " + str(ds)
        if not self.interruptible:
            assert(ds.min <= self.min and ds.max >= self.max), "(1) " + str(self) + " ∩ " + str(ds)
            return DurationSpec(self.min, self.max, False)
        elif not ds.interruptible:
            assert(self.min <= ds.min and self.max >= ds.max), "(2) " + str(self) + " ∩ " + str(ds)
            return DurationSpec(ds.min, ds.max, False)
        else:
            new_min = max(self.min, ds.min)
            new_max = min(self.max, ds.max)
            assert(new_min <= new_max), "(3) " + str(self) + " ∩ " + str(ds)
            return DurationSpec(new_min, new_max, True)

    def implements(self, ds):
        same = self.interruptible == ds.interruptible
        if self.interruptible:
            return same and self.min <= ds.min and self.max >= ds.max
        else:
            return same and self.min >= ds.min and self.max <= ds.max
