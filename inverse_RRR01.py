#CODE FOR INVERSE KINEMATICS RRR ROBOT
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

L1=0.400
L2=0.300
L3=0.200
robot= rtb.DHRobot(
[
	rtb.RevoluteDH(d=L1, a=0, alpha=np.pi/2, qlim=[-2.61, 2.61]),
	rtb.RevoluteDH(d=0, a=L2, alpha=0, qlim=[-2.61, 2.61]),
	rtb.RevoluteDH(d=0, a=L3, alpha=0, qlim=[-2.61, 2.61]),
], name="RRR", base=SE3(0,0,0), tool=SE3())
print(robot)

robot.plot(q=[0, 0, 0], limits=[-0.9,0.9,-0.9,0.9,-0.1,1.0], backend='pyplot', shadow=True, jointaxes=True, block=True)

# #We validate the DH with forward kinematics
# #To change the angles conveniently
joint1 = 45  #In degrees
joint2 = 30
joint3 = 60

T=robot.fkine([np.deg2rad(joint1), np.deg2rad(joint2), np.deg2rad(joint3)])
print(T)
Tn = np.array(T).astype(np.float64) #Convert to numpy

print(f"The angles in the forward kinematics are ({joint1:.2f},{joint2:.2f},{joint3:.2f})")
print(f"The coordinate is ({Tn[0, 3]:.3f},{Tn[1, 3]:.3f},{Tn[2, 3]:.3f}), which we pass to the inverse...")

#Inverse kinematics
#Coordinates we want to reach
x= round(Tn[0, 3], 3)
y= round(Tn[1, 3], 3)
z= round(Tn[2, 3], 3)

# #The equations we derived geometrically
theta1 = np.rad2deg(np.arctan2(y,x)) #Output in degrees

r2 =np.sqrt((x**2)+(y**2))
s =z-L1
r = np.sqrt((r2**2)+(s**2))
phi2=np.arctan2(s,r2) #phi2=np.arctan(y/x)
phi1=np.arccos(((L2**2)+(r**2)-(L3**2))/(2*L2*r))
theta2=np.rad2deg(phi2-phi1) #Output in degrees

phi3=np.arccos(((L2**2)+(L3**2)-(r**2))/(2*L2*L3))
theta3=180-np.rad2deg(phi3) #Output in degrees

print(f"The joint variables are ({theta1:.3f}, {theta2:.3f}, {theta3:.3f})")
