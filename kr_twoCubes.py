import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot

# Configurar formato de impresión
np.set_printoptions(suppress=True, precision=4,
                   formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

# ====================================================
# CONFIGURACIÓN DE LOS ROBOTS (igual que antes)
# ====================================================

dh_params = [
    rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=np.deg2rad([-155, 155])),
    rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=np.deg2rad([-180, 65])),
    rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=np.deg2rad([-110, 170])),
    rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=np.deg2rad([-165, 165])),
    rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=np.deg2rad([-140, 140])),
    rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=np.deg2rad([-360, 360]))
]

radio = 1.53
traslape = 0.4
distancia = 2 * radio * (1 - traslape)

robot1 = rtb.DHRobot(dh_params.copy(), name="KR5-1", base=SE3(0, 0, 0))
robot2 = rtb.DHRobot(dh_params.copy(), name="KR5-2", base=SE3(distancia, 0, 0))

for robot in [robot1, robot2]:
    robot.tool = SE3.OA([0, 1, 0], [0, 0, 1])
    robot.qz = np.zeros(6)
    robot.qhome = [0, np.deg2rad(45), np.deg2rad(90), 0, np.deg2rad(45), 0]

# ====================================================
# FUNCIÓN MODIFICADA PARA DIRECCIONES OPUESTAS
# ====================================================

def generar_trayectoria_cubo(robot, centro_relativo, tamaño, reverse=False):
    """
    Genera trayectoria en forma de cubo
    - reverse=True: invierte el sentido del recorrido
    """
    x_base, y_base, z_base = robot.base.t
    x_c = x_base + centro_relativo[0]
    y_c = y_base + centro_relativo[1]
    z_c = z_base + centro_relativo[2]

    # Definición original del cubo
    waypoints = [
        (x_c, y_c-tamaño, z_c-tamaño, [180, 36, -81]),  # A
        (x_c, y_c-tamaño, z_c, [0, 0, -46]),            # B
        (x_c, y_c, z_c, [0, 0, -46]),                   # C
        (x_c, y_c, z_c-tamaño, [180, 36, -81]),         # D
        (x_c, y_c-tamaño, z_c-tamaño, [180, 36, -81]),  # A
        (x_c+tamaño, y_c-tamaño, z_c-tamaño, [180, 36, -81]),  # H
        (x_c+tamaño, y_c-tamaño, z_c, [0, 0, -46]),     # G
        (x_c+tamaño, y_c, z_c, [0, 0, -46]),            # F
        (x_c+tamaño, y_c, z_c-tamaño, [180, 36, -81]),  # E
        (x_c+tamaño, y_c-tamaño, z_c-tamaño, [180, 36, -81]),  # H
        (x_c+tamaño, y_c-tamaño, z_c, [0, 0, -46]),     # G
        (x_c, y_c-tamaño, z_c, [0, 0, -46]),            # B
        (x_c, y_c, z_c, [0, 0, -46]),                   # C
        (x_c+tamaño, y_c, z_c, [0, 0, -46]),            # F
        (x_c+tamaño, y_c, z_c-tamaño, [180, 36, -81]),  # E
        (x_c, y_c, z_c-tamaño, [180, 36, -81])          # D
    ]

    # Invertir el orden si se requiere
    if reverse:
        waypoints = waypoints[::-1]
        # Ajustar rotaciones para dirección inversa
        for i in range(len(waypoints)):
            rpy = waypoints[i][3]
            waypoints[i] = (*waypoints[i][:3], [rpy[0], rpy[1], (rpy[2] + 180) % 360])

    # Construir trayectoria
    T_cubo = []
    for wp in waypoints:
        T = SE3(wp[0], wp[1], wp[2]) * SE3.RPY(wp[3], unit='deg')
        T_cubo.append(T)

    # Generar segmentos
    traj = []
    for i in range(len(T_cubo)-1):
        segment = rtb.ctraj(T_cubo[i], T_cubo[i+1], 10)
        traj.extend(segment)

    # Resolver IK
    qtraj = []
    q0 = robot.qz

    for pose in traj:
        sol = robot.ikine_LM(pose, q0=q0, tol=1e-5, ilimit=500, slimit=25)
        if not sol.success:
            raise ValueError(f"Pose no alcanzable en {pose.t}")
        qtraj.append(sol.q)
        q0 = sol.q

    return np.array(qtraj)

# ====================================================
# GENERACIÓN DE TRAYECTORIAS OPUESTAS
# ====================================================
tamaño_cubo = 1.2
centro_relativo = np.array([-0.6, 0.6, 1.3])

print("Generando trayectoria para robot 1 (dirección normal)...")
qtraj1 = generar_trayectoria_cubo(robot1, centro_relativo, tamaño_cubo)

print("\nGenerando trayectoria para robot 2 (dirección invertida)...")
qtraj2 = generar_trayectoria_cubo(robot2, centro_relativo, tamaño_cubo, reverse=True)

# ====================================================
# VISUALIZACIÓN (igual que antes)
# ====================================================
backend = PyPlot()
backend.launch()
backend.add(robot1, eeframe=True, shadow=True)
backend.add(robot2, eeframe=True, shadow=True)

limites = [
    min(robot1.base.t[0], robot2.base.t[0]) - 1,
    max(robot1.base.t[0], robot2.base.t[0]) + 1,
    -1, 1,
    -0.15, 2.5
]
backend.ax.set(xlim=limites[:2], ylim=limites[2:4], zlim=limites[4:6])
backend.ax.set_box_aspect([1,1,1])

try:
    for i in range(max(len(qtraj1), len(qtraj2))):
        if i < len(qtraj1):
            robot1.q = qtraj1[i]
        if i < len(qtraj2):
            robot2.q = qtraj2[i]

        backend.step()
        plt.pause(0.05)

except KeyboardInterrupt:
    print("\nAnimación interrumpida")

backend.hold()