#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from spatialmath import *
from spatialmath.base import *
from math import pi
np.set_printoptions(
    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

from sympy import *
from sympy import Symbol,Matrix, simplify
from sympy.matrices import rot_axis3

theta1, L1, theta2, L2 = symbols('theta1 L1 theta2 L2')

# Referencia T0
T0 = rotz(0, unit='deg')
trplot(T0, dims=[-1, 1, -1, 1, -1, 1], color='k') #Origen

T01 = trotz(theta1) @ transl(L1, 0, 0)
print (f"Primera transformación:\n {T01}\n")

T12 = trotz(theta2) @ transl(L2, 0, 0)
print (f"Segunda transformación:\n {T12}\n")

T02 = T01 @ T12 #Transformación completa
print (f"Transformación completa:\n {T02}\n")

#Convertir el ndarray a una matriz de sympy
M = Matrix(T02)
#Simplificar la matriz
M_simplified = M.applyfunc(simplify)

#Mejor visualización
def nice_print_matrix(matrix):
    return '\n'.join([' '.join([str(entry.evalf()) for entry in row]) for row in matrix.tolist()])

#Imprimir la matriz simplificada
print(nice_print_matrix(M_simplified))
print('\n')

#Sustituir y resolver operaciones
M_evaluated = M_simplified.subs({theta1: np.deg2rad(30), L1: 4, theta2: np.deg2rad(0), L2: 3})
print(nice_print_matrix(M_evaluated))