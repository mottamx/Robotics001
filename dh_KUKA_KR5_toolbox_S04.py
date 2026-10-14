import numpy as np
# For the example where we generate the DH matrix
from spatialmath import *
from spatialmath.base import *
# To use DH
import roboticstoolbox as rtb

#Now using DH with the Toolbox *************
# Units: meters and radians (the DH table is in mm and degrees)
# Joint limits (qlim): datasheet ranges converted to the DH variable q
# q1:+-155, q2:-155 to +90, q3:-80 to +200, q4:+-165, q5:+-140, q6: unlimited (+-360)
kr5 = rtb.DHRobot(
[
    rtb.RevoluteDH(d=0.400, a=0.180, alpha=np.pi/2, qlim=[-2.705, 2.705]),
    rtb.RevoluteDH(d=0,     a=0.600, alpha=0,       offset=np.pi/2, qlim=[-2.705, 1.571]), #theta2 = q2 + 90
    rtb.RevoluteDH(d=0,     a=0.120, alpha=np.pi/2, qlim=[-1.396, 3.491]),
    rtb.RevoluteDH(d=0.620, a=0,     alpha=-np.pi/2, qlim=[-2.880, 2.880]),
    rtb.RevoluteDH(d=0,     a=0,     alpha=np.pi/2, qlim=[-2.443, 2.443]),
    rtb.RevoluteDH(d=0.115, a=0,     alpha=0,       qlim=[-2*np.pi, 2*np.pi])
], name="Kuka KR5", base=SE3(0,0,0))
print(kr5)

# To modify the angles conveniently
joint1 = np.deg2rad(0)
joint2 = np.deg2rad(-45) #The +90 offset is already in the robot definition
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0)
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)

T06DH = kr5.fkine([joint1, joint2, joint3, joint4, joint5, joint6])
print(T06DH)

T06DH_all = kr5.fkine_all([joint1, joint2, joint3, joint4, joint5, joint6])
# print(T06DH_all[1]) #T01
# print(T06DH_all[2]) #T02
# print(T06DH_all[3]) #T03
# print(T06DH_all[4]) #T04
# print(T06DH_all[5]) #T05
# print(T06DH_all[6]) #T06

# Define joint variables
q=np.array([[0, 0, 0, 0, 0, 0],
            [joint1, 0, 0, 0, 0, 0],
            [joint1, joint2, 0, 0, 0, 0],
            [joint1, joint2, joint3, 0, 0, 0],
            [joint1, joint2, joint3, joint4, 0, 0],
            [joint1, joint2, joint3, joint4, joint5, 0],
            [joint1, joint2, joint3, joint4, joint5, joint6],
            [joint1, joint2, joint3, joint4, joint5, 0],
            [joint1, joint2, joint3, joint4, 0, 0],
            [joint1, joint2, joint3, 0, 0, 0],
            [joint1, joint2, 0, 0, 0, 0],
            [joint1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]])

# Plot the q positions, one every 3 seconds
kr5.plot(q=q, backend='pyplot', dt=3, limits=[-1.2,1.2,-1.2,1.2,-0.2,1.6], shadow=True, jointaxes=True)

# Plot and save video
# kr5.plot(q=q, backend='pyplot', dt=3, limits=[-1.2,1.2,-1.2,1.2,-0.2,1.6], shadow=True, loop=False, jointaxes=True, movie="robot.gif")

# Plot with an interactive controller
# q1=np.array([[0, 0, 0, 0, 0, 0]])
# kr5.teach(q1)
