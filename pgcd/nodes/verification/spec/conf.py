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

dRealJobs = 4 #TODO not more than 1 when using docker!
dRealTimeout = 120
