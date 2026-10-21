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

L1=0.400 #We will use these values
L2=0.300
robot= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0, a=L1, alpha= 0, qlim=[-2.61, 2.61]),
	rtb.RevoluteDH(d=0, a=L2, alpha=0, qlim=[-2.61, 2.61]),
], name="2Links", base=SE3(0,0,0), tool=SE3())
print(robot)
tool=SE3()  # Zero end effector pose (no tool offset)

#We validate the DH with forward kinematics
#To change the angles conveniently
joint1 = 30 #In degrees
joint2 = 0

T=robot.fkine([np.deg2rad(joint1), np.deg2rad(joint2)], tool=None)
# print(T)
Tn = np.array(T).astype(np.float64) #Convert to numpy
robot.plot(q=[np.deg2rad(joint1), np.deg2rad(joint2)], limits=[-0.8,0.8,-0.8,0.8,-0.1,0.2], backend='pyplot', shadow=True, jointaxes=True, block=True)

print(f"The angles in the forward kinematics are ({joint1:.2f},{joint2:.2f})")
print(f"The coordinate is ({Tn[0, 3]:.3f},{Tn[1, 3]:.3f}), which we pass to the inverse...")

t = T.t
# print(t)
q = SE3([round(t[0],4), round(t[1],4), 0])
# print(q)

#Inverse kinematics
we=[1, 1, 1, 0, 0, 0]
sol = robot.ikine_LM(Tep=q, q0=[0,0], ilimit=5000, slimit=1000, tol=0.000005, joint_limits=True, mask=we)
print(sol)

# #Show result
if sol.success:
	# #We extract the angles from the result
	theta1=np.rad2deg(sol.q[0])
	theta2=np.rad2deg(sol.q[1])
	print(f"The joint variables are ({theta1:.3f}, {theta2:.3f})")
 	# Create a new figure
	robot.plot(q=sol.q, limits=[-0.8,0.8,-0.8,0.8,-0.1,0.2], backend='pyplot', shadow=True, jointaxes=True, block=True)
	#If we pass them again...
	T2=robot.fkine(sol.q)
	Tn2 = np.array(T2).astype(np.float64) #Convert to numpy
	print(f"The coordinate is ({Tn2[0, 3]:.3f},{Tn2[1, 3]:.3f}), with the new angles")
# print(T)
else:
    	print(f'Could not obtain a solution {sol.success}')
