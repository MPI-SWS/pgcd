from sympy.vector import CoordSys3D

# general configuration options and default values for the verification

#global frame (used as default when no other frame is avail)
worldFrame = CoordSys3D('World')

# default bounds for the footprint
minX = -10
maxX =  10
minY = -10
maxY =  10
minZ = 0
maxZ = 2


# FIXME disable duration check while the thing is not fully done
enableDurationCheck = False

# Quite expensive, so do only when introducing new components
enableProcessAbstractionCheck = False

# Quite expensive, so disable for testing the other VCs
enableFPCheck = True
# Quite expensive, so disable for testing
enableMPincludeFPCheck = True #TODO for MP which reuse the componenet FP, this is not needed

dRealJobs = 6
dRealTimeout = 3600
dRealPrecision = 0.01

## https://stackoverflow.com/questions/51412465/python-best-way-to-setup-global-logger-and-set-the-logging-level-from-command-li
import logging
import sys
# from argparse import ArgumentParser
# parser = ArgumentParser()
# parser.add_argument("-v", "--verbose", help="Increase output verbosity", action="store_const", const=logging.DEBUG, default=logging.INFO)
# args = parser.parse_args()
# logging.basicConfig(level = args.verbose)
print("running", sys.argv)
if "-v" in sys.argv or "--verbose" in sys.argv:
    logging.basicConfig(level = logging.DEBUG)
else:
    logging.basicConfig(level = logging.INFO)
#TODO dreal as opts
