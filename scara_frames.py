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

#Ahora usando el DH con Toolbox *************

scara= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0.160, a=0.350, alpha= 0, qlim=[-2.58, 2.58]),
	rtb.RevoluteDH(d=0, 	a=0.300, alpha=0, qlim=[-2.61, 2.61]),
	rtb.PrismaticDH(theta=0, a=0, alpha=np.pi, offset=0.21, qlim=[-0.370, 0.0]),
	rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-6.28, 6.28])
], name="SR6iA", base=SE3(0,0,0))
print(scara)

#Para modificar los angulos comodamente
joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)
joint3 = 0 #Valor prismatico offset +0.210
joint4 = np.deg2rad(0)

T04DH=scara.fkine([joint1, joint2, joint3, joint4]) #Lo definimos asi por el offset
#print(T04DH)

T04DH_all = scara.fkine_all([joint1, joint2, joint3, joint4])
print(T04DH_all[1]) #T01
# print(T04DH_all[2]) #T12
# print(T04DH_all[3]) #T23
# print(T04DH_all[4]) #T34

trplot(T04DH_all[1], color='red', frame='0', length=0.7)

plt.grid(True)
plt.axis('equal')
plt.show()