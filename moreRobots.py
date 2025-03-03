import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3
from roboticstoolbox.backends.PyPlot import PyPlot
env=PyPlot() #Declaramos este ambiente para poder graficar dos o mas robots
#Declaracion del robot1
robot1= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0.160, a=0.350, alpha= 0, qlim=[-2.58, 2.58]),
	rtb.RevoluteDH(d=0, 	a=0.300, alpha=0, qlim=[-2.61, 2.61]),
	rtb.PrismaticDH(theta=0, a=0, alpha=np.pi, offset=0.21, qlim=[0, 0.210]),
	rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-6.28, 6.28])
], name="SCARA", base=SE3(0,0,0))
#Declaracion del robot2
robot2= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0.160, a=0.350, alpha= 0, qlim=[-2.58, 2.58]),
	rtb.RevoluteDH(d=0, 	a=0.300, alpha=0, qlim=[-2.61, 2.61]),
	rtb.PrismaticDH(theta=0, a=0, alpha=np.pi, offset=0.21, qlim=[0, 0.210]),
	rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-6.28, 6.28])
], name="SCARA2", base=SE3(0,0.3,0))

#Declaramos un vector de variables articulares para cada robot
q=np.array([[0, -np.pi/2, 0.0, 0],
            [0, -np.pi/2, -0.37, 0],
           [0, 0, 0.0, 0],
           [0, 0, -0.37, 0]])

q2=np.array([[0, np.pi/2, 0.0, 0],
            [0, np.pi/2, -0.37, 0],
           [0, 0, 0.0, 0],
           [0, 0, -0.37, 0]])

#Esta forma de graficar solo es para más de un robot a la vez
env.launch()
env.add(robot1) #Añadimos un robot
env.add(robot2) #Añadimos el otro robot
env.step(3) #Le decimos que cada paso (del vector de posiciones toma 2 segundos)
#Este ciclo for va a extraer cada vector de valores
for i,j in zip(q,q2):
    robot1.q=i
    robot2.q=j
    env.step(3)

env.hold()

