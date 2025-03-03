import sympy as sp
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot

# Configurar NumPy para suprimir la notación científica y limitar la precisión
np.set_printoptions(suppress=True, precision=4,
                    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

# Función para dibujar una esfera de trabajo
def dibujar_esfera(ax, base_pos, radio, color='r', alpha=0.1):
    # El centro está en el eje de la primera articulación, a 400mm de altura desde la base
    centro = [base_pos[0], base_pos[1], base_pos[2] + 0.4]

    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    x = centro[0] + radio * np.outer(np.cos(u), np.sin(v))
    y = centro[1] + radio * np.outer(np.sin(u), np.sin(v))
    z = centro[2] + radio * np.outer(np.ones(np.size(u)), np.cos(v))

    return ax.plot_surface(x, y, z, color=color, alpha=alpha)

# Crear el primer robot explícitamente
robot1 = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
        rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(65)]),
        rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=[np.deg2rad(-110), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-165), np.deg2rad(165)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-140), np.deg2rad(140)]),
        rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Kuka KR5 #1", base=SE3(0, 0, 0))

# Configurar el primer robot
robot1.tool = SE3.OA([0, 1, 0], [0, 0, 1])
robot1.qz = [np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0)]
robot1.qhome = [np.deg2rad(0.0), np.deg2rad(45.0), np.deg2rad(90.0), np.deg2rad(0.0), np.deg2rad(45.0), np.deg2rad(0.0)]

# Calcular la posición del segundo robot para un traslape del 40%
radio = 1.53  # Radio de la esfera de trabajo (1530mm)
traslape = 0.4  # 40% de traslape
distancia = 2 * radio * (1 - traslape)  # Distancia entre los centros de las esferas

# Crear el segundo robot explícitamente
robot2 = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
        rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(65)]),
        rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=[np.deg2rad(-110), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-165), np.deg2rad(165)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-140), np.deg2rad(140)]),
        rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Kuka KR5 #2", base=SE3(0,distancia, 0))

# Configurar el segundo robot
robot2.tool = SE3.OA([0, 1, 0], [0, 0, 1])
robot2.qz = [np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0)]
robot2.qhome = [np.deg2rad(45.0), np.deg2rad(45.0), np.deg2rad(90.0), np.deg2rad(0.0), np.deg2rad(45.0), np.deg2rad(0.0)]

# Crear una instancia del backend PyPlot
backend = PyPlot()
backend.launch()

# Añadir los robots al gráfico
backend.add(robot1, robot1.qhome)
backend.add(robot2, robot2.qhome)

# Obtener el eje para añadir elementos adicionales
ax = backend.ax

# Definir las posiciones base de los robots
base_pos1 = [0, 0, 0]  # Posición base del primer robot
base_pos2 = [0,distancia, 0]  # Posición base del segundo robot

# Dibujar las esferas de trabajo usando la función
dibujar_esfera(ax, base_pos1, radio, color='r', alpha=0.1)  # Esfera roja para el primer robot
dibujar_esfera(ax, base_pos2, radio, color='g', alpha=0.1)  # Esfera verde para el segundo robot

# Actualizar los límites del gráfico para asegurar que se vean ambos robots y sus esferas
ax.set_xlim([-radio-0.2, radio+0.2])
ax.set_ylim([-radio-0.2, distancia+radio+0.2])
ax.set_zlim([0, 2*radio])

# Actualizar el gráfico
backend.step()

# Mantener la figura abierta
backend.hold()