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

L1=0.400 #Vamos a usar estos valores
L2=0.300
robot= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0, a=L1, alpha= 0, qlim=[-2.58, 2.58]),
	rtb.RevoluteDH(d=0, a=L2, alpha=0, qlim=[-2.62, 2.62]),
], name="2Links", base=SE3(0,0,0), tool=SE3())
# print(robot)
#tool=SE3()  # Pose de efector final cero

#Validamos con la directa el DH
#Para modificar los angulos comodamente
joint1 = 30 #En grados
joint2 = 0

T=robot.fkine([np.deg2rad(joint1), np.deg2rad(joint2)], tool=None)
# print(T)
Tn = np.array(T).astype(np.float64) #Convertir a numpy
robot.plot(q=[np.deg2rad(joint1), np.deg2rad(joint2)], limits=[-0.8,0.8,-0.8,0.8,-0.1,0.2], backend='pyplot', shadow=True, jointaxes=True, block=True)

print(f"Los ángulos en la Cdirecta son ({joint1:.2f},{joint2:.2f})")
print(f"La coordenada es ({Tn[0, 3]:.3f},{Tn[1, 3]:.3f}), que pasamos a la inversa...")

t = T.t
# print(t)
q = SE3([round(t[0],4), round(t[1],4), 0])
# print(q)

#Cinemática inversa
we=[1, 1, 1, 0, 0, 0]
sol = robot.ikine_LM(Tep=q, q0=[0,0], ilimit=5000, slimit=1000, tol=0.000005, joint_limits=True, mask=we)
print(sol)

# #Mostrar resultado
if sol.success:
	# #Extraemos los ángulos del resultado
	theta1=np.rad2deg(sol.q[0])
	theta2=np.rad2deg(sol.q[1])
	print(f"Las variables articulares son ({theta1:.3f}, {theta2:.3f})")
 	# Crea una nueva figura
	robot.plot(q=sol.q, limits=[-0.8,0.8,-0.8,0.8,-0.1,0.2], backend='pyplot', shadow=True, jointaxes=True, block=True)
	#Si los volvemos a pasar...
	T2=robot.fkine(sol.q)
	Tn2 = np.array(T2).astype(np.float64) #Convertir a numpy
	print(f"La coordenada es ({Tn2[0, 3]:.3f},{Tn2[1, 3]:.3f}), con los nuevos angulos")
# print(T)
else:
    	print(f'No se pudo obtener solución {sol.success}')