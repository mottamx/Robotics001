import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot
np.set_printoptions(suppress=True, precision=4,
                    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

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

robot.qz = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]

# Cargar la fuente
font = rtb.rtb_load_jsonfile("data/hershey.json")

word = "IMT"
x_offset = 0  # Inicialmente no hay traslación en x
lift = 0.1  # Altura de la pluma
scale = 0.25  # Escala 10cm
# Modificación en la generación de coordenadas para dibujar en plano XZ
via = np.empty((0, 3))

# Definir la posición inicial en el espacio XZ
x_start = 0.0  # Distancia inicial en X desde la base del robot
y_fixed = -0.8  # Distancia fija en Y (profundidad) cuando está escribiendo
z_start = 0.8  # Altura inicial en Z

x_offset = x_start  # Inicializar el offset en X
z_offset = z_start  # Inicializar el offset en Z
retract = 0.05  # Distancia de retracción en Y (alejándose del plano)

for a_letter in word:
    letter = font[a_letter]
    for stroke in letter["strokes"]:
        xyz = np.array(stroke) * scale  # Aplicar escala al stroke
        # Convertir de XY a XZ (intercambiar Y por Z)
        temp_xyz = np.zeros((xyz.shape[0], 3))
        temp_xyz[:, 0] = xyz[:, 0] + x_offset  # X se mantiene igual pero con offset
        temp_xyz[:, 1] = y_fixed  # Y fija (distancia desde el robot) cuando está escribiendo
        temp_xyz[:, 2] = xyz[:, 1] + z_offset  # Z toma el valor que antes era Y, con offset

        # Retraer pluma antes de moverse al siguiente trazo (alejarse del plano)
        if via.shape[0] > 0:
            via = np.vstack((via, [via[-1, 0], via[-1, 1] + retract, via[-1, 2]]))
        # Agregar los puntos del trazo
        via = np.vstack((via, temp_xyz))
        # Retraer pluma después del trazo (alejarse del plano)
        via = np.vstack((via, [temp_xyz[-1, 0], temp_xyz[-1, 1] + retract, temp_xyz[-1, 2]]))
    # Actualizar el desplazamiento en x para la siguiente letra
    x_offset = temp_xyz[-1, 0] + 0.05

# Convertir en array
via2 = np.array(via)

# Modificar la trayectoria para que el movimiento sea en el plano XZ
xyz_traj = rtb.mstraj(via2, qdmax=[0.25, 0.25, 0.25], q0=[x_start, y_fixed + retract, z_start], dt=0.02, tacc=0.2, verbose=False).q

# Modificar la orientación del efector final
T_pen2 = SE3.Trans(xyz_traj) * SE3.OA([1, 0, 0], [0, -1, 0])

# Mantener la posición inicial original (codo arriba)
q_inicial = np.array([0, np.deg2rad(-45), np.deg2rad(00), np.deg2rad(-30), np.deg2rad(0), 0])

# Calcular cinemática inversa
sol = robot.ikine_LM(T_pen2, q0=q_inicial, ilimit=100, slimit=100, tol=1e-6)
print(f"Éxito: {sol.success}")

if not sol.success:
    print("No se pudo encontrar una solución para la cinemática inversa")
    exit()

# Inicializar el entorno PyPlot
env = PyPlot()
env.launch()

# Configurar los límites de visualización
env.ax.set_xlim([-1.5, 1.5])
env.ax.set_ylim([-1.5, 1.5])
env.ax.set_zlim([-0.1, 1.3])

# Añadir el robot al entorno
env.add(robot)

# Lista para almacenar los puntos de la trayectoria
traj_points = []

# Configurar el robot en la posición inicial
robot.q = sol.q[0]
env.step(1)

# Acceder al objeto de figura y ejes de matplotlib
fig = env.fig
ax = env.ax

# Crear un plano para representar la superficie de escritura
# Definir las coordenadas del plano (un rectángulo en el plano XZ)
# y_plane = y_fixed  # Posición fija en Y donde se escribe
# x_min, x_max = -0.5, 1.5  # Límites en X (ajustar según sea necesario)
# z_min, z_max = 0.5, 1.2   # Límites en Z

# Crear los vértices del plano
# X, Z = np.meshgrid([x_min, x_max], [z_min, z_max])
# Y = np.ones_like(X) * y_plane
# ax.plot_surface(X, Y, Z, alpha=0.2, color='gray')

# Recorrer la trayectoria y actualizar la visualización
line_obj = None
writing_points = []  # Para almacenar solo los puntos donde está escribiendo
current_writing = False  # Flag para saber si está escribiendo o retraído

for i in range(len(sol.q)):
    # Actualizar la configuración del robot
    robot.q = sol.q[i]

    # Obtener la posición actual del efector final
    T_current = robot.fkine(robot.q)
    current_pos = T_current.t
    traj_points.append(current_pos)

    # Determinar si está escribiendo o retraído
    is_writing = abs(current_pos[1] - y_fixed) < 0.01

    # Si cambia de estado (escribiendo/retraído), actualizar el flag
    if is_writing != current_writing:
        current_writing = is_writing
        if current_writing:
            # Comienza un nuevo trazo
            writing_points = []

    # Si está escribiendo, añadir a los puntos de escritura
    if current_writing:
        writing_points.append(current_pos)

        # Dibujar el trazo actual
        if len(writing_points) > 1:
            points_array = np.array(writing_points)
            ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2], 'b-', linewidth=2)

    # Convertir la lista de puntos a un array numpy para graficar la trayectoria completa
    if len(traj_points) > 1:
        points_array = np.array(traj_points)

        # Si ya existe una línea, eliminarla
        if line_obj:
            line_obj[0].remove()

        # Crear una nueva línea con todos los puntos acumulados usando matplotlib directamente
        line_obj = ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2], 'r-', linewidth=1, alpha=0.5)

    # Actualizar la visualización
    env.step(0.01)  # Tiempo entre actualizaciones

# Mantener la visualización al final
print("Trayectoria completada")
env.hold()