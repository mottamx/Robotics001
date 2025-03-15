import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot

# Configurar NumPy para suprimir la notación científica y limitar la precisión
np.set_printoptions(suppress=True, precision=4,
                   formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

# ====================================================
# CONFIGURACIÓN DE LOS ROBOTS
# ====================================================

# Calcular la posición del segundo robot para un traslape del 40%
radio = 1.53  # Radio de la esfera de trabajo (1530mm)
traslape = 0.4  # 40% de traslape
distancia = 2 * radio * (1 - traslape)  # Distancia entre los centros de las esferas

# Configuración de los robots
dh_params = [
    rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
    rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(65)]),
    rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=[np.deg2rad(-110), np.deg2rad(170)]),
    rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-165), np.deg2rad(165)]),
    rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-140), np.deg2rad(140)]),
    rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)])
]

# Crear robots con bases separadas
robot1 = rtb.DHRobot(dh_params.copy(), name="KR5-1", manufacturer="Kuka", base=SE3(0, 0, 0))
robot2 = rtb.DHRobot(dh_params.copy(), name="KR5-2", manufacturer="Kuka", base=SE3(distancia, 0, 0))

# Configurar los robots
for robot in [robot1, robot2]:
    robot.tool = SE3.OA([0, 1, 0], [0, 0, 1])
    robot.qz = np.zeros(6)

# ====================================================
# GENERACIÓN DE TRAYECTORIAS PARA AMBOS ROBOTS
# ====================================================

def generar_trayectoria_cubo(robot, centro_cubo, tamaño, invertir=False):
    """
    Genera trayectoria en forma de cubo para un robot
    invertir: si es True, se recorre el cubo en orden inverso
    """
    # Ajustar coordenadas relativas a la base del robot
    x_c, y_c, z_c = centro_cubo + robot.base.t

    # Definir los puntos del cubo
    T_cubo = [
        SE3(x_c, y_c-tamaño, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg'),  # A
        SE3(x_c, y_c-tamaño, z_c) * SE3.RPY([0, 0, -46], unit='deg'),            # B
        SE3(x_c, y_c, z_c) * SE3.RPY([0, 0, -46], unit='deg'),                    # C
        SE3(x_c, y_c, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg'),         # D
        SE3(x_c, y_c-tamaño, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg'),  # A
        SE3(x_c+tamaño, y_c-tamaño, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg'),  # H
        SE3(x_c+tamaño, y_c-tamaño, z_c) * SE3.RPY([0, 0, -46], unit='deg'),     # G
        SE3(x_c+tamaño, y_c, z_c) * SE3.RPY([0, 0, -46], unit='deg'),            # F
        SE3(x_c+tamaño, y_c, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg'),  # E
        SE3(x_c+tamaño, y_c-tamaño, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg'),  # H
        SE3(x_c+tamaño, y_c-tamaño, z_c) * SE3.RPY([0, 0, -46], unit='deg'),     # G
        SE3(x_c, y_c-tamaño, z_c) * SE3.RPY([0, 0, -46], unit='deg'),            # B
        SE3(x_c, y_c, z_c) * SE3.RPY([0, 0, -46], unit='deg'),                    # C
        SE3(x_c+tamaño, y_c, z_c) * SE3.RPY([0, 0, -46], unit='deg'),            # F
        SE3(x_c+tamaño, y_c, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg'),  # E
        SE3(x_c, y_c, z_c-tamaño) * SE3.RPY([180, 36, -81], unit='deg')          # D
    ]

    # Invertir el orden si se solicita
    if invertir:
        T_cubo = list(reversed(T_cubo))

    # Generar trayectoria cartesiana
    print(f"Generando trayectoria cartesiana para {robot.name}...")
    traj = []
    for i in range(len(T_cubo)-1):
        segment = rtb.ctraj(T_cubo[i], T_cubo[i+1], 10)
        traj.extend(segment)

    # Resolver cinemática inversa
    print(f"Resolviendo cinemática inversa para {robot.name}...")
    qtraj = []
    total_poses = len(traj)
    q0 = robot.qz

    for i, pose in enumerate(traj):
        # Mostrar progreso en la misma línea
        animation = "|/-\\"
        print(f"\rProgreso {robot.name}: {i+1}/{total_poses} ({(i+1)/total_poses*100:.1f}%) {animation[i%4]}", end="")

        sol = robot.ikine_LM(
            pose,
            q0=q0,
            tol=1e-5,
            ilimit=500,
            slimit=25,
        )

        if sol.success:
            qtraj.append(sol.q)
            q0 = sol.q  # Actualizar q0 para la siguiente iteración
        else:
            print(f"\nError: Pose no alcanzable en paso {i+1} para {robot.name}: {pose.t}")
            raise ValueError(f"No se pudo alcanzar la pose {i+1} para {robot.name}. Cancelando ejecución.")

    print(f"\n{robot.name}: Trayectoria completada")
    return np.array(qtraj)

# Tamaño y posición de los cubos para cada robot
tamaño_cubo = 1.0  # Tamaño del cubo (1 metro)

# Centro del cubo para cada robot (relativo a sus bases)
centro_cubo1 = np.array([-0.5, 0.5, 1.2])
centro_cubo2 = np.array([-0.5, 0.5, 1.2])

# Calcular trayectorias para ambos robots
qtraj1 = generar_trayectoria_cubo(robot1, centro_cubo1, tamaño_cubo, invertir=False)  # Robot 1: sentido normal
qtraj2 = generar_trayectoria_cubo(robot2, centro_cubo2, tamaño_cubo, invertir=True)   # Robot 2: sentido inverso

# ====================================================
# VISUALIZACIÓN Y ANIMACIÓN
# ====================================================

# Inicializar backend y configurar la visualización
backend = PyPlot()
backend.launch()

# Añadir robots al entorno
backend.add(robot1, eeframe=True, shadow=True)
backend.add(robot2, eeframe=True, shadow=True)

# Configurar límites de visualización para que se vean ambos robots
limites = [-2, distancia+2, -2, 2, -0.5, 2]
backend.ax.set_xlim(limites[:2])
backend.ax.set_ylim(limites[2:4])
backend.ax.set_zlim(limites[4:6])

# Asegurar que la vista no esté distorsionada
backend.ax.set_box_aspect([limites[1]-limites[0], limites[3]-limites[2], limites[5]-limites[4]])

# Preparar la visualización para trayectoria
# Calcular posiciones de efector final para dibujar las trayectorias
traj_points1 = []
traj_points2 = []

for q in qtraj1:
    traj_points1.append(robot1.fkine(q).t)

for q in qtraj2:
    traj_points2.append(robot2.fkine(q).t)

traj_points1 = np.array(traj_points1)
traj_points2 = np.array(traj_points2)

# Dibujar las trayectorias completas (opcional)
backend.ax.plot(traj_points1[:, 0], traj_points1[:, 1], traj_points1[:, 2], 'r-', linewidth=1, alpha=0.5)
backend.ax.plot(traj_points2[:, 0], traj_points2[:, 1], traj_points2[:, 2], 'b-', linewidth=1, alpha=0.5)

# Animar el movimiento de los robots
print("Iniciando animación...")
try:
    # Encontrar la longitud máxima de las trayectorias
    max_len = max(len(qtraj1), len(qtraj2))

    # Variables para seguimiento de la trayectoria actual
    current_points1 = []
    current_points2 = []
    line_obj1 = None
    line_obj2 = None

    for i in range(max_len):
        # Actualizar la configuración de cada robot si hay puntos disponibles
        if i < len(qtraj1):
            robot1.q = qtraj1[i]
            pos1 = robot1.fkine(robot1.q).t
            current_points1.append(pos1)

            # Actualizar el dibujo de la trayectoria actual del robot 1
            if len(current_points1) > 1:
                points_array = np.array(current_points1)
                if line_obj1:
                    line_obj1[0].remove()
                line_obj1 = backend.ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2],
                                          'r-', linewidth=2)

        if i < len(qtraj2):
            robot2.q = qtraj2[i]
            pos2 = robot2.fkine(robot2.q).t
            current_points2.append(pos2)

            # Actualizar el dibujo de la trayectoria actual del robot 2
            if len(current_points2) > 1:
                points_array = np.array(current_points2)
                if line_obj2:
                    line_obj2[0].remove()
                line_obj2 = backend.ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2],
                                          'b-', linewidth=2)

        # Actualizar visualización
        backend.step()

        # Mostrar progreso
        print(f"\rProgreso animación: {i+1}/{max_len} ({(i+1)/max_len*100:.1f}%)", end="")

        # Controlar velocidad de animación
        plt.pause(0.05)

    print("\nAnimación completada")

except KeyboardInterrupt:
    print("\nAnimación interrumpida por el usuario")

# Mantener la visualización abierta
backend.hold()