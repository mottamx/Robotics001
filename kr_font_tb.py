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
np.set_printoptions(suppress=True, precision=4,
                    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

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
font = rtb.rtb_load_jsonfile("data/hershey.json")

word = "IMT"
x_offset = 0  # Inicialmente no hay traslación en x
lift = 0.1  # Altura de la pluma
scale = 0.125  # Escala 10cm
via = np.empty((0, 3))

for a_letter in word:
    letter = font[a_letter]
    for stroke in letter["strokes"]:
        xyz = np.array(stroke) * scale  # Aplicar escala al stroke
        xyz[:, 0] += x_offset  # Aplicar desplazamiento en x
        xyz = np.pad(xyz, ((0, 0), (0, 1)))  # Añadir columna de z=0
        # Levantar pluma antes de moverse al siguiente trazo
        if via.shape[0] > 0:
            via = np.vstack((via, [via[-1, 0], via[-1, 1], lift]))
        # Agregar los puntos del trazo
        via = np.vstack((via, xyz))
        # Levantar pluma después del trazo
        via = np.vstack((via, [xyz[-1, 0], xyz[-1, 1], lift]))
    # Actualizar el desplazamiento en x para la siguiente letra
    x_offset = xyz[-1, 0] + 0.05

# Convertir en array
via2 = np.array(via)

xyz_traj = rtb.mstraj(via2, qdmax=[0.25, 0.25, 0.25], q0=[0, 0, lift], dt=0.02, tacc=0.2, verbose=False).q
T_pen2 = SE3.Trans(0.20, 0, 0.0) * SE3.Trans(xyz_traj) * SE3.OA([0, 1, 0], [0, 0, -1])

q_inicial = np.array([0, np.deg2rad(45), np.deg2rad(90), 0, np.deg2rad(0), 0])

# Calcular cinemática inversa
sol = robot.ikine_LM(T_pen2,q0=q_inicial, ilimit=100, slimit=100, tol=1e-6)
print(f"Éxito: {sol.success}")

# Mostrar resultado
robot.plot(sol.q, limits=[-0.8, 0.8, -0.8, 0.8, -0.1, 1],
          backend='pyplot', shadow=True, jointaxes=True, block=True)