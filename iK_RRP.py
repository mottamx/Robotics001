#CODE FOR INVERSE KINEMATICS RRP ROBOT
import sympy as sp
from sympy.matrices import rot_axis3
# For the example where we generate the DH matrix
from spatialmath import *
from spatialmath.base import *
# To be able to plot
import matplotlib.pyplot as plt
import numpy as np
#To use DH
import roboticstoolbox as rtb

L1=0.800
L2=0.600
L3=0.100
robotRRP= rtb.DHRobot(
[
    rtb.RevoluteDH(d=L1, a=0, alpha=np.pi/2, qlim=[-2.61, 2.61]),
    rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=np.pi/2, qlim=[-1.39, 1.39]),
    rtb.PrismaticDH(theta=0, a=0, alpha=0, offset=L2+L3, qlim=[0.0, 0.2]),
], name="RRP", base=SE3(0,0,0), tool=SE3())
print(robotRRP)

#We validate the DH with forward kinematics
#To change the angles conveniently
joint1 = 45  #In degrees
joint2 = 0
joint3 = 0.0 #Distance (m)

robotRRP.plot(q=[np.deg2rad(joint1), np.deg2rad(joint2), joint3],\
    limits=[-0.9,0.9,-0.9,0.9,-0.1,1.0], backend='pyplot', shadow=True, jointaxes=True, block=True)

T=robotRRP.fkine([np.deg2rad(joint1), np.deg2rad(joint2), joint3])
print(T)
Tn = np.array(T).astype(np.float64) #Convert to numpy
print(f"The joint variables in the forward kinematics are ({joint1:.2f},{joint2:.2f},{joint3:.2f})")
print(f"The coordinate is ({Tn[0, 3]:.3f},{Tn[1, 3]:.3f},{Tn[2, 3]:.3f}), which we pass to the inverse...")

#Inverse kinematics
t = T.t
#print(t)
q = SE3([round(t[0],3), round(t[1],3), round(t[2],3)]) #Position only: with 3 DOF we cannot also impose the orientation
print(f"Target:\n {q}")

# Inverse kinematics function
we=[1, 1, 1, 0, 0, 0] #position only: x, y, z (no orientation)
sol = robotRRP.ikine_LM(Tep=q, q0=[0,0,0], ilimit=3000, slimit=50, tol=1e-8, joint_limits=True, mask=we)
print(sol)

# #Show result
if sol.success:
    # #We extract the angles from the result
    theta1=np.rad2deg(sol.q[0])
    theta2=np.rad2deg(sol.q[1])
    d3=sol.q[2]

    print(f"The joint variables are ({theta1:.3f}, {theta2:.3f}, {d3:.3f})")
    # Create a new figure
    robotRRP.plot(q=sol.q, limits=[-0.9,0.9,-0.9,0.9,-0.1,1.0], backend='pyplot', shadow=True, jointaxes=False, block=True)
    #If we pass them again...
    T2=robotRRP.fkine(sol.q)
    Tn2 = np.array(T2).astype(np.float64) #Convert to numpy
    print(f"The coordinate is ({Tn2[0, 3]:.3f},{Tn2[1, 3]:.3f},{Tn2[2, 3]:.3f}), with the new angles")
# print(T)
else:
    print(f'Could not obtain a solution {sol.success}')
