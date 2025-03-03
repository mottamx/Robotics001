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
np.set_printoptions(suppress=True, precision=4)

robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
        rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(65)]),
        rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=[np.deg2rad(-110), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-165), np.deg2rad(165)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-140), np.deg2rad(140)]),
        rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Kuka KR5", base=SE3(0,0,0))
print(robot)

robot.qz=[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]
# robot.teach(robot.qz)

T_inicial = robot.fkine(robot.qz)
# print(f"Matriz de posición inicial:\n{T_inicial}")

# # Crear una nueva matriz de transformación destino
# R_down = np.array([
#     [1, 0, 0],    # X apunta hacia adelante, dirección original
#     [0, 1, 0],    # Y apunta hacia la derecha, dirección original
#     [0, 0, -1]    # Z apunta hacia abajo, dirección invertida
# ])
#Roll 180 en X, Pitch 0 en Y, Yaw 0 en Z
T_destino = SE3.Trans(0.848, -0.0, 0.28) * SE3(rpy2tr(180.0, 0.142, 0.0, 'deg'))
# print("Matriz T_destino:\n", T_destino, "\n")

#Para mantener codo arriba
q_inicial = np.array([0, np.deg2rad(45), np.deg2rad(90), 0, np.deg2rad(0), 0])
robot.plot(q_inicial, block=True)

q = robot.ikine_LM(T_destino,q0=q_inicial, ilimit=100, slimit=100, tol=1e-6)
print(f"Éxito: {q.success}")

if q.success:
    print(f"Solución de cinemática inversa: {q.q}")
    T_resultado = robot.fkine(q.q)
    print(f"Matriz resultante después de IK:\n{T_resultado}")
    # Visualizar el robot en la posición calculada
    robot.plot(q.q, block=True)