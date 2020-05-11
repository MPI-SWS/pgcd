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

dRealJobs = 4
dRealTimeout = 240
dRealPrecision = 0.01
