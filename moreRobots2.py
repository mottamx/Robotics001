import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3
from roboticstoolbox.backends.PyPlot import PyPlot
env=PyPlot() #Declaramos este ambiente para poder graficar dos o mas robots
#Declaracion del robot1
robot1= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0.4, a=.18, alpha=np.pi/2, qlim=[-2.705, 2.705]),
    rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[-np.pi, 1.134]),
    rtb.RevoluteDH(d=0, a=0.12, alpha=np.pi/2, qlim=[-0.2618, 2.757]),
    rtb.RevoluteDH(d=0.62, a=0, alpha=-np.pi/2, qlim=[-6.10, 6.10]),
    rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[-2.268, 2.268]),
    rtb.RevoluteDH(d=0.05, a=0, alpha=0, qlim=[-6.10, 6.10]),
    ], name="Robot1 KR5", base=SE3(-0.5,-0.5,0))
#Declaracion del robot2
robot2= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0.4, a=.18, alpha=np.pi/2, qlim=[-2.705, 2.705]),
    rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[-np.pi, 1.134]),
    rtb.RevoluteDH(d=0, a=0.12, alpha=np.pi/2, qlim=[-0.2618, 2.757]),
    rtb.RevoluteDH(d=0.62, a=0, alpha=-np.pi/2, qlim=[-6.10, 6.10]),
    rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[-2.268, 2.268]),
    rtb.RevoluteDH(d=0.05, a=0, alpha=0, qlim=[-6.10, 6.10]),
    ], name="Robot1 KR5", base=SE3(0.25, 0.75,0))

#Declaramos un vector de variables articulares para cada robot
q=np.array([[0, 0, 0, 0, 0, 0],
            [-np.pi/2, 0, 0, 0, 0, 0],
            [-np.pi/2, 0, np.pi/3,  0, 0, 0],
            [-np.pi/2, 0, 0, 0, 0, 0],
            [-np.pi/2, 0, np.pi/3, 0, 0, 0],
            [-np.pi/2, 0, 0, 0, 0, 0]])

q2=np.array([[0, 0, 0, 0, 0, 0],
            [np.pi/2, 0, 0, 0, 0, 0],
            [np.pi/2, 0, np.pi/3,  0, 0, 0],
            [np.pi/2, 0, 0, 0, 0, 0],
            [np.pi/2, 0, np.pi/3, 0, 0, 0],
            [np.pi/2, 0, 0, 0, 0, 0]])

#Esta forma de graficar solo es para más de un robot a la vez
env.launch(limits=[-1,1,-1,1,0,2])
env.add(robot1) #Añadimos un robot
env.add(robot2) #Añadimos el otro robot
env.step(3) #Le decimos que cada paso (del vector de posiciones toma 2 segundos)
#Este ciclo for va a extraer cada vector de valores
for i,j in zip(q,q2):
    robot1.q=i
    robot2.q=j
    env.step(3)

env.hold()

