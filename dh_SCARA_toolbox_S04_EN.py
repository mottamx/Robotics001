import sympy as sp
from sympy.matrices import rot_axis3
# For the example where we generate the DH matrix
from spatialmath import *
from spatialmath.base import *
# For plotting
import matplotlib.pyplot as plt
import numpy as np
# To use DH
import roboticstoolbox as rtb

#Now using DH with the Toolbox *************

scara= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0.160, a=0.350, alpha= 0, qlim=[-2.58, 2.58]),
	rtb.RevoluteDH(d=0, 	a=0.300, alpha=0, qlim=[-2.61, 2.61]),
	rtb.PrismaticDH(theta=0, a=0, alpha=np.pi, offset=0.21, qlim=[-0.370, 0.0]),
	rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-6.28, 6.28])
], name="SR6iA", base=SE3(0,0,0))
print(scara)

# To modify the angles conveniently
joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)
joint3 = -0.210 #Prismatic value, offset +0.210
joint4 = np.deg2rad(0)

T04DH=scara.fkine([joint1, joint2, joint3, joint4]) #We define it this way because of the offset
print(T04DH)

T04DH_all = scara.fkine_all([joint1, joint2, joint3, joint4])
print(T04DH_all[1]) #T01
print(T04DH_all[2]) #T12
print(T04DH_all[3]) #T23
print(T04DH_all[4]) #T34

# Define joint variables
q=np.array([[0, 0, 0, 0],
            [joint1, 0, 0, 0],
            [joint1, joint2, 0, 0],
            [joint1, joint2, joint3, 0],
            [joint1, joint2, joint3, joint4],
            [joint1, joint2, joint3, 0],
            [joint1, joint2, 0, 0],
            [joint1, 0, 0, 0],
            [0, 0, 0, 0]])

# Plot the q positions, one every 3 seconds
scara.plot(q=q, backend='pyplot',dt=3, limits=[-0.8,0.8,-0.8,0.8,-0.4,0.6], shadow=True, jointaxes=True)

# Plot and save video
# scara.plot(q=q, backend='pyplot',dt=3, limits=[-0.8,0.8,-0.8,0.8,-0.4,0.6], shadow=True, loop=False, jointaxes=True, movie="robot.gif")

# Plot with an interactive controller
q1=np.array([[0, 0, 0, 0]])
scara.teach(q1)


