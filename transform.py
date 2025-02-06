import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt

from spatialmath import *
from spatialmath.base import *

# from sympy import Symbol, Matrix

theta_deg = 30
theta_rad = np.deg2rad(theta_deg)

R = rot2(theta_rad)

print(R)

R2=trot2(theta_rad)

trplot2(R)
plt.axis('equal')
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Rotation by 30 degrees')
plt.show()