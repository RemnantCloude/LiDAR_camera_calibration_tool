#  Cartesian transformation between two coordinate systems
from math import cos, sin, radians
import numpy as np

# Degrees --> radians
def trig(angle):
  r = radians(angle)
  return cos(r), sin(r)

#  Returns the transformation matrix
def matrix(rotation=(0,0,0), translation=(0,0,0)): 
  Cx, Sx = trig(rotation[0])
  Cy, Sy = trig(rotation[1])
  Cz, Sz = trig(rotation[2])
  dX = translation[0]
  dY = translation[1]
  dZ = translation[2]

  return np.array([[Cz*Cy, Cz*Sy*Sx-Sz*Cx , Cz*Sy*Cx+Sz*Sx, dX],    # Rotation done in the order z, y', x''  
     [Sz*Cy, Sz*Sy*Sx+Cz*Cx, Sz*Sy*Cx-Cz*Sx, dY],
     [-Sy, Cy*Sx, Cy*Cx, dZ],
     [0, 0, 0, 1]])