import sympy as sp


class BBox:
    """Bounding box"""

    def __init__(self, minX = -10, maxX = 10, minY = 10, maxY = 10, minZ = 0, maxZ = 2):
        self.minX = minX
        self.maxX = maxX
        self.minY = minY
        self.maxY = maxY
        self.minZ = minZ
        self.maxZ = maxZ

    def contains(self, px, py, pz):
        return sp.And(px >= self.minX, px <= self.maxX, py >= self.minY, py <= self.maxY, pz >= self.minZ, pz <= self.maxZ)
