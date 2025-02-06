import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3
#Declaracion del robot
robot1= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0.160, a=0.350, alpha= 0, qlim=[-2.58, 2.58]),
	rtb.RevoluteDH(d=0, 	a=0.300, alpha=0, qlim=[-2.61, 2.61]),
	rtb.PrismaticDH(theta=0, a=0, alpha=np.pi, offset=0.21, qlim=[-0.370, 0.0]),
	rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-6.28, 6.28])
], name="SCARA", base=SE3(0,0,0))
#Mostrar tabla DH
#print(robot)
q=[0,0,0,0]
#Funcion para que genere sliders para cada articulación
robot1.teach(q, limits=[-0.8,0.8,-0.8,0.8,0,0.6])
