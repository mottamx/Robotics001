import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot
np.set_printoptions(suppress=True, precision=4)

# Crear el robot Kuka KR5
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
        rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(65)]),
        rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=[np.deg2rad(-110), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-165), np.deg2rad(165)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-140), np.deg2rad(140)]),
        rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Kuka KR5", base=SE3(0,0,0))
print(robot)

# Configuración inicial
q_inicial = np.array([0, np.deg2rad(45), np.deg2rad(90), 0, np.deg2rad(0), 0])
T_inicial = robot.fkine(q_inicial)

# Configuración destino
T_destino = SE3.Trans(0.848, -0.0, 0.28) * SE3(rpy2tr(180.0, 0.142, 0.0, 'deg'))

# Calcular la cinemática inversa para la posición destino
q_final = robot.ikine_LM(T_destino, q0=q_inicial, ilimit=100, slimit=100, tol=1e-6)

if not q_final.success:
    print("No se pudo encontrar una solución para la cinemática inversa")
    exit()

print(f"Solución de cinemática inversa: {q_final.q}")
T_resultado = robot.fkine(q_final.q)
print(f"Matriz resultante después de IK:\n{T_resultado}")

# Generar trayectoria en el espacio de articulaciones
n_steps = 50  # Número de pasos en la trayectoria
traj = rtb.jtraj(q_inicial, q_final.q, n_steps)

# Inicializar el entorno PyPlot
env = PyPlot()
env.launch()

# Añadir el robot al entorno
env.add(robot)

# Lista para almacenar los puntos de la trayectoria
traj_points = []

# Configurar el robot en la posición inicial
robot.q = q_inicial
env.step(1)

# Acceder al objeto de figura y ejes de matplotlib
fig = env.fig
ax = env.ax

# Recorrer la trayectoria y actualizar la visualización
line_obj = None
for i in range(n_steps):
    # Actualizar la configuración del robot
    robot.q = traj.q[i]

    # Obtener la posición actual del efector final
    T_current = robot.fkine(robot.q)
    current_pos = T_current.t
    traj_points.append(current_pos)

    # Convertir la lista de puntos a un array numpy para graficar
    if len(traj_points) > 1:
        points_array = np.array(traj_points)

        # Si ya existe una línea, eliminarla
        if line_obj:
            line_obj[0].remove()

        # Crear una nueva línea con todos los puntos acumulados usando matplotlib directamente
        line_obj = ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2], 'r-', linewidth=2)

    # Actualizar la visualización
    env.step(0.1)  # Tiempo entre actualizaciones

# Mantener la visualización al final
print("Trayectoria completada")
env.hold()