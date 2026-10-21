import sympy as sp
from sympy.matrices import rot_axis3
# For the example where we generate the DH matrix
from spatialmath import *
from spatialmath.base import *
# To be able to plot
import matplotlib.pyplot as plt
import numpy as np
# To use DH
import roboticstoolbox as rtb

L1=0.400 #We will use these values
L2=0.300
robot= rtb.DHRobot(
[
	rtb.RevoluteDH(d=0, a=L1, alpha= 0, qlim=[-2.61, 2.61]),
	rtb.RevoluteDH(d=0, a=L2, alpha=0, qlim=[-2.61, 2.61]),
], name="2Links", base=SE3(0,0,0))
print(robot)

q1=np.array([[0, 0]])
robot.teach(q1)

#We validate the DH with forward kinematics
#To change the angles conveniently
joint1 = 110 #In degrees
joint2 = -90

T=robot.fkine([np.deg2rad(joint1), np.deg2rad(joint2)])
print(T)
Tn = np.array(T).astype(np.float64) #Convert to numpy

print(f"The angles in the forward kinematics are ({joint1:.2f},{joint2:.2f})")
print(f"The coordinate is ({Tn[0, 3]:.3f},{Tn[1, 3]:.3f}), which we pass to the inverse...")

#Inverse kinematics
#Coordinates we want to reach
x= round(Tn[0, 3], 3)
y= round(Tn[1, 3], 3)

#The equations we derived geometrically
r1=np.sqrt((x**2)+(y**2))

phi1=np.arccos(((L1**2)+(r1**2)-(L2**2))/(2*L1*r1))

phi2=np.arctan2(y,x) #phi2=np.arctan(y/x)

theta1=np.rad2deg(phi2-phi1) #Output in degrees

phi3=np.arccos(((L2**2)+(L1**2)-(r1**2))/(2*L2*L1))

theta2=180-np.rad2deg(phi3) #Output in degrees

print(f"The joint variables are ({theta1:.3f}, {theta2:.3f})")

theta1_up = np.rad2deg(phi2+phi1)
theta2_up = -theta2
print(f"The joint variables for the other solution are ({theta1_up:.3f}, {theta2_up:.3f})")