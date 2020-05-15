from sympy import *

def linear(var, min, max, duration):
    return var * (max - min) / duration + min

def smoothstep(var, min, max, duration):
    x = var / duration
    slope = max - min
    return slope * (x**2 * 3 - x**3 * 2) + min
