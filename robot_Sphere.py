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

# Declaramos nuestro robot, incluidos limites de movimiento
robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
        rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(65)]),
        rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=[np.deg2rad(-110), np.deg2rad(170)]),
        rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-165), np.deg2rad(165)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-140), np.deg2rad(140)]),
        rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Kuka KR5", base=SE3(0,0,0))

# Ponemos el TCP alineado con el brazo
robot.tool = SE3.OA([0, 1, 0], [0, 0, 1])
robot.qz = [np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0)]  # Valores el cero
robot.qhome=[np.deg2rad(0.0), np.deg2rad(45.0), np.deg2rad(90.0), np.deg2rad(0.0), np.deg2rad(45.0), np.deg2rad(0.0)]  # Valores el cero

# Verificar que quedo igual el DH
print(robot)
# robot.plot(q=robot.qz, eeframe=True, jointaxes=False, shadow=True ,backend='pyplot', block=True)
# robot.teach(q=robot.qhome)

# Crear una instancia del backend PyPlot
backend = PyPlot()
backend.launch()

# Añadir el robot al gráfico
backend.add(robot, robot.qhome)

# Obtener el eje para añadir elementos adicionales
ax = backend.ax

# Definir el centro y radio de la esfera basado en la datasheet
# El centro está en el eje de la primera articulación, a 400mm de altura
centro = [0, 0, 0.4]  # La base está a 400mm (0.4m) según la datasheet
radio = 1.53  # 1530mm (1.53m) de alcance máximo

# Crear una esfera (representada como puntos en la superficie)
u = np.linspace(0, 2 * np.pi, 30)
v = np.linspace(0, np.pi, 30)
x = centro[0] + radio * np.outer(np.cos(u), np.sin(v))
y = centro[1] + radio * np.outer(np.sin(u), np.sin(v))
z = centro[2] + radio * np.outer(np.ones(np.size(u)), np.cos(v))

# Añadir la esfera al gráfico con transparencia para ver el robot dentro
ax.plot_surface(x, y, z, color='r', alpha=0.1)

# Actualizar los límites del gráfico para asegurar que se vea toda la esfera
ax.set_xlim([-radio-0.2, radio+0.2])
ax.set_ylim([-radio-0.2, radio+0.2])
ax.set_zlim([0, 2*radio])

# Actualizar el gráfico
backend.step()

# Mantener la figura abierta
backend.hold()