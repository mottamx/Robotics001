import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
np.set_printoptions(suppress=True, precision=4,
                    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

#Aqui importarian su robot, no este
robot = rtb.models.DH.IRB140()
print(robot)

# Configuración inicial (posición home)
q_home = robot.qz  # Tienen que asignar un vector de ceros

# Mostrar el robot en modo teach
robot.teach(q_home, backend='pyplot')
