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
backend.add(robot)
robot.q=[np.deg2rad(-1.8), np.deg2rad(-90), np.deg2rad(77.7), np.deg2rad(0), np.deg2rad(-1.6), np.deg2rad(0.0)]
backend.step()

# Obtener el eje para añadir elementos adicionales
ax = backend.ax

# Definir el centro y radio del círculo basado en la datasheet
centro = [0, 0]  # Centro del círculo en el plano XY
radio_max = 1.53  # 1530mm (1.53m) de alcance máximo

# Crear un círculo en el plano XY para el alcance máximo
theta = np.linspace(0, 2 * np.pi, 100)
x_max = centro[0] + radio_max * np.cos(theta)
y_max = centro[1] + radio_max * np.sin(theta)
z_max = np.ones_like(theta) * 0.4  # Altura de la primera articulación

# Añadir el círculo al gráfico
ax.plot(x_max, y_max, z_max, 'r-', linewidth=2, label='Alcance máximo')

# Configurar la vista para que sea similar a la vista superior del datasheet
ax.view_init(elev=90, azim=-90)  # Vista desde arriba (plano XY)

# Actualizar los límites del gráfico
ax.set_xlim([-radio_max-0.2, radio_max+0.2])
ax.set_ylim([-radio_max-0.2, radio_max+0.2])

# Actualizar el gráfico
backend.step()

# Mantener la figura abierta
backend.hold()