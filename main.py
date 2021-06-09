import pygame
import sys
from operator import itemgetter
from point import Point3D

class Simulation:
    def __init__(self, win_width=640, win_height=480):
        pygame.init()

        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("3D wireframe Cube Simulation")

        self.clock = pygame.time.Clock()

        self.vertices = [
            Point3D(-1,  1, -1),
            Point3D( 1,  1, -1),
            Point3D( 1, -1, -1),
            Point3D(-1, -1, -1),
            Point3D(-1,  1,  1),
            Point3D( 1,  1,  1),
            Point3D( 1, -1,  1),
            Point3D(-1, -1,  1)]

        # Define: 1. the vertices that compose each of the 6 faces
        #         2. the color of each face
        self.faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]
        self.colors = [(255,0,255),(255,0,0),(0,255,0),(0,0,255),(0,255,255),(255,255,0)]
        self.angleX, self.angleY, self.angleZ = 0, 0, 0

    def run(self):
        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
            self.clock.tick(50)
            self.screen.fill((0,0,0))
            # Will hold transformed vertices.
            t = []
            for v in self.vertices:
                # Rotate the point around X axis, then around Y axis, and finally around Z axis.
                r = v.rotateX(self.angleX).rotateY(self.angleY).rotateZ(self.angleZ)
                # Transform the point from 3D to 2D
                p = r.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                # Put the point in the list of transformed vertices
                t.append(p)
            avg_z = []
            i = 0
            for f in self.faces:
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z) / 4.0
                avg_z.append([i, z])
                i = i + 1

            for tmp in sorted(avg_z, key=itemgetter(1), reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index] # face vertices
                pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
                             (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
                             (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
                             (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y),]
                pygame.draw.polygon(self.screen, self.colors[face_index], pointlist)
            pygame.display.flip()

if __name__ == "__main__":
    Simulation().run()