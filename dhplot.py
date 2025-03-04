import sympy as sp #1.13.3
from sympy.matrices import rot_axis3
# Para el ejemplo donde generamos la matriz DH
from spatialmath import * #1.1.14
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt #3.8.3
import numpy as np #1.26.4

# Definir los símbolos
theta, d, a, alpha = sp.symbols('theta, d, a, alpha')

T = sp.Matrix([
    [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
    [sp.sin(theta), sp.cos(theta)* sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
    [0, sp.sin(alpha), sp.cos(alpha), d],
    [0, 0, 0, 1]
])
#sp.pprint(T)

#Definimos los angulos para que cada uno sea unico
theta_1, theta_2,theta_3, theta_4,theta_5, theta_6  = \
    sp.symbols('theta_1, theta_2,theta_3, theta_4,theta_5, theta_6')

#De la tabla
T01 = T.subs({d: 0.680, a: 0.200, alpha: -sp.pi/2})
T01 = T01.subs({theta: theta_1})
# sp.pprint(T01)
T12 = T.subs({d: 0, a: 0.890, alpha: 0}) #theta2-sp.pi/2
T12 = T12.subs({theta: theta_2})
# sp.pprint(T12)
T23 = T.subs({d: 0, a: 0.150, alpha: -sp.pi/2})
T23 = T23.subs({theta: theta_3})
# sp.pprint(T23)
T34 = T.subs({d: 0.880, a: 0, alpha: sp.pi/2})
T34 = T34.subs({theta: theta_4})
# sp.pprint(T34)
T45 = T.subs({d: 0, a: 0, alpha: -sp.pi/2})
T45 = T45.subs({theta: theta_5})
# sp.pprint(T45)
T56 = T.subs({d: 0.140, a: 0, alpha: 0})
T56 = T56.subs({theta: theta_6})
# sp.pprint(T56)

T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
# T06_s= T06.applyfunc(sp.simplify)
#sp.pprint(T06_s)

#Para modificar los angulos comodamente
joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0) - sp.pi/2 #Offset
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0)
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)

T06_solved = T06.subs({theta_1: joint1, theta_2: joint2,theta_3: joint3,theta_4: joint4,theta_5: joint5,theta_6: joint6})
# sp.pprint(T06_solved)

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

T23_n = T23.subs({theta_3: joint3})
T03_n = T02_n @ T23_n
T03_n = np.array(T03_n).astype(np.float64)
trplot(T03_n, length=0.7, color='g', frame='3')

T34_n = T34.subs({theta_4:joint4})
T04_n = T03_n @ T34_n
T04_n = np.array(T04_n).astype(np.float64)
trplot(T04_n, length=0.7, color='m', frame='4')

T45_n = T45.subs({theta_5: joint5})
T05_n = T04_n @ T45_n
T05_n = np.array(T05_n).astype(np.float64)
trplot(T05_n, length=0.7, color='y', frame='5')

T56_n = T56.subs({theta_6: joint6})
T06_n = T05_n @ T56_n
T06_n = np.array(T06_n).astype(np.float64)
trplot(T06_n, length=0.7, color='c', frame='6')

T06_n = np.array(T06_solved).astype(np.float64)
trplot(T06_n, length=0.7, color='purple', frame='f', width=2.0)

plt.grid(True)
plt.title('Brazo iRB4400')
plt.axis('equal')
plt.show()

tranimate(T06_n, length=0.7, repeat=False, interval=100, wait=True)


