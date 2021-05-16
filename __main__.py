from numpy.lib.index_tricks import MGridClass
import pygame
from scipy.spatial import Delaunay
from pygame.locals import *
import numpy as np

# PyGame settings
screen = (350, 350)
fps = 30

# Simulation settings
wind_speed = 1
wind_direction = 0

wind = np.array([wind_speed * np.cos(wind_direction), wind_speed * np.sin(wind_direction)])

class PhysicsObject(object):

    def __init__(self, mass, points):
        self.mass = mass
        self.set_points(points)
        self.pos = np.array([0, 0], dtype = float)
        self.vel = np.array([0, 0], dtype=float)
        self.acc = np.array([0, 0], dtype=float)

    def set_points(self, points):
        if len(points) == 3:
            points.insert(1, ((points[0][0] + points[1][0])/2 , (points[0][1] + points[1][1])/2) )
        self.delaunay = Delaunay(points)
        self.points = self.delaunay.points
        self.triangles = self.delaunay.points[self.delaunay.simplices]
        
        self.moi = 0
        self.com = np.zeros(2)
        total_area = 0
        
        for p in self.triangles:
            area = np.cross(p[1]-p[0], p[2]-p[0])
            centriod = np.mean(p, axis=0)
            self.com += centriod * area
            self.moi += np.dot(centriod, centriod) * area
            total_area += area
        self.com /= total_area
        self.moi *= self.mass / total_area
        self.moi -= self.mass * np.dot(self.com, self.com)


    def render(self, surface):
        if self.points.__len__() < 3:
            return
        pygame.draw.polygon(surface, (124, 124, 124), [p + self.pos for p in self.points])
        pygame.draw.aalines(surface, (0, 0, 0), True, [p + self.pos for p in self.points])
        pygame.draw.circle(surface, (255, 0, 0), self.com, np.sqrt(self.moi/10))
    
    def update(self, dt):
        self.pos += 1/2 * self.acc * dt**2 + self.vel * dt
        self.vel += self.acc * dt

# square = PhysicsObject(100, [(0, 0), (100, 0), (100, 100), (0, 100)])

# Lauch PyGame
pygame.init()
timer = pygame.time.Clock()
window = pygame.display.set_mode( screen )
pygame.display.set_caption( "TESTING OUT PYGAME!")
mouse = pygame.mouse

points = []
triangles = []
objects = []

done = False
while not done:
    dt = timer.tick(fps)

    for event in pygame.event.get():
        if event.type == QUIT:
            done = True
        elif event.type == MOUSEBUTTONUP:
            points.append(mouse.get_pos())
        elif event.type == KEYDOWN:
            objects.append(PhysicsObject(10, points))
            points = []
        
    
    window.fill((0, 0, 0))
    for object in objects:
        object.render(window)
        object.update(dt)
    
    if len(points) > 2:
        pygame.draw.polygon(window, (124, 124, 124), points)
        pygame.draw.aalines(window, (0, 0, 0), True, points)

    pygame.display.update()