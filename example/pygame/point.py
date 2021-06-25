import math

class Point3D:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def rotateX(self, angle):
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x, y, z = self.x, self.y, self.z
        y = y*cosa - z*sina
        z = y*sina + z*cosa
        return Point3D(x, y, z)

    def rotateY(self, angle):
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x, y, z = self.x, self.y, self.z
        z = z*cosa - x*sina
        x = z*sina + x*cosa
        return Point3D(x, y, z)

    def rotateZ(self, angle):
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x, y, z = self.x, self.y, self.z
        x = x*cosa - y*sina
        y = x*sina + y*cosa
        return Point3D(x, y, z)

    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, 1)

    def __str__(self):
        return f'x, y, z = {self.x, self.y, self.z}'