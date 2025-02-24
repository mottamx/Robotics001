import sympy as sp
from sympy.matrices import rot_axis3
# Para el ejemplo donde generamos la matriz DH
from spatialmath import *
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt
import numpy as np

#Para usar el DH
import roboticstoolbox as rtb

# Definir los símbolos
theta, d, a, alpha = sp.symbols('theta, d, a, alpha')
# T = RzTzTxRx
T = sp.Matrix([
    [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
    [sp.sin(theta), sp.cos(theta)* sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
    [0, sp.sin(alpha), sp.cos(alpha), d],
    [0, 0, 0, 1]
])
#sp.pprint(T)
#Definimos los angulos y distancias para que cada uno sea unico
theta_1, theta_2,d_3, theta_4 = \
    sp.symbols('theta_1, theta_2, d_3, theta_4')

#De la tabla
T01 = T.subs({d: 0.160, a: 0.350, alpha: 0})
T01 = T01.subs({theta: theta_1})
# sp.pprint(T01)

T12 = T.subs({d: 0, a: 0.300, alpha: 0})
T12 = T12.subs({theta: theta_2})
# sp.pprint(T12)

T23 = T.subs({theta: 0, a: 0.0, alpha: sp.pi})
T23 = T23.subs({d: d_3}) #Recordar el -0.210
# sp.pprint(T23)

T34 = T.subs({d: 0.0, a: 0, alpha: 0})
T34 = T34.subs({theta: theta_4})
# sp.pprint(T34)

T04 = T01 @ T12 @ T23 @ T34
T04_s= T04.applyfunc(sp.simplify)
# sp.pprint(T04_s)

# #Para modificar los angulos comodamente
joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)
joint3 = 0 #Valor prismatico offset +0.210
joint4 = np.deg2rad(0)

T04_solved = T04.subs({theta_1: joint1, theta_2: joint2, d_3: joint3,theta_4: joint4})
sp.pprint(T04_solved)

# Graficar frames *************
# Referencia TOrigen
T0 = rotz(0, unit='deg')
trplot(T0, length=0.7, color='k', frame='0')

T01_n = T01.subs({theta_1: joint1})
T01_n = np.array(T01_n).astype(np.float64)
trplot(T01_n, length=0.7, color='b', frame='1')

T12_n = T12.subs({theta_2: joint2})
T02_n = T01_n @ T12_n
T02_n = np.array(T02_n).astype(np.float64)
trplot(T02_n, length=0.7, color='r', frame='2')

T23_n = T23.subs({d_3: joint3})
T03_n = T02_n @ T23_n
T03_n = np.array(T03_n).astype(np.float64)
trplot(T03_n, length=0.7, color='g', frame='3')

T34_n = T34.subs({theta_4:joint4})
T04_n = T03_n @ T34_n
T04_n = np.array(T04_n).astype(np.float64)
trplot(T04_n, length=0.7, color='m', frame='4')

Tf_n = np.array(T04_solved).astype(np.float64)
trplot(Tf_n, length=0.7, color='purple', frame='f', width=2.0)

plt.grid(True)
plt.title('Fanuc SR6iA SCARA')
plt.axis('equal')
plt.show(block=True)
