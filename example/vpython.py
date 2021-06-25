from vpython import *
import math

a = box(length=5, width=3, height=1)
while True:
    rate(100)
    a.rotate(axis=(1, 0, 0), angle=math.pi/400, origin=(0, 0, 0))