import sympy as sp
import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3, SO3
import roboticstoolbox as rtb
np.set_printoptions(
    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})




robot = rtb.models.DH.IRB140()
print(robot)
robot.plot(q=robot.qd, limits=[-0.8,0.8,-0.8,0.8,-0.1,1],\
    backend='pyplot', shadow=True, jointaxes=True, block=True)

print(robot.fkine(robot.qd))

#We define poses in space for the EE
T0 = SE3.Trans([-0.35, 0, 0.45]) * SE3.Rx(3)
T1 = SE3.Trans([-0.35, 0.25, 0.45]) * SE3.Rx(2)

#We solve their inverse kinematics to obtain the joint values
sol1=robot.ikine_LM(T0, tol=0.025, joint_limits=True)
print(sol1)
robot.plot(q=sol1.q, limits=[-0.8,0.8,-0.8,0.8,-0.1,1],\
    backend='pyplot', shadow=True, jointaxes=True, block=True)

sol2=robot.ikine_LM(T1, tol=0.025, joint_limits=True)
print(sol2)
robot.plot(q=sol2.q, limits=[-0.8,0.8,-0.8,0.8,-0.1,1],\
    backend='pyplot', shadow=True, jointaxes=True, block=True)

t = np.arange(0, 2, 0.05) #Array of time values
#from 0 to 2, in 50 ms steps
print(t)
# traj = rtb.mtraj(rtb.trapezoidal, sol1.q, sol2.q, t)
traj = rtb.jtraj(sol1.q, sol2.q, t) #Joint Trajectory
#Remember that the trajectory has q, qd, qdd

robot.plot(q=traj.q, limits=[-0.8,0.8,-0.8,0.8,-0.1,1], eeframe=True, \
    backend='pyplot', shadow=True, jointaxes=True, block=True)

print(traj.q)
print(len(traj.q))

rtb.xplot(t, traj.q, labels="Joint 1, Joint 2, Joint 3, Joint 4, Joint 5, Joint 6", block=True)

#To get the Cartesian pose (forward kinematics)
T = robot.fkine(traj.q)
#We extract the translation
p = T.t
rtb.xplot(t, T.t, labels="x y z", block=True);
rtb.xplot(t, T.rpy("xyz"), labels="roll pitch yaw",block=True)



#orientation
R0 = SO3.Rz(-1) * SO3.Ry(-1) #initial
R1 = SO3.Rz(1) * SO3.Ry(1) #final
#Extracting RPY
rpy0 = R0.rpy()
rpy1 = R1.rpy()
#Generate the trajectory
trajRPY=rtb.mtraj(rtb.quintic, rpy0, rpy1, 50)
trajRPY.plot(block=True)
pose = SO3.RPY(trajRPY.q) #And convert it back to a matrix
pose.animate().save('RPY_Traj.gif')

#Cartesian motion
T0 = SE3.Trans([0.4, 0.2, 0]) * SE3.RPY(0, 0, 3)
T1 = SE3.Trans([-0.4, -0.2, 0.3]) * SE3.RPY(-np.pi/4, np.pi/4, -np.pi/2)
#pose between T0 and T1 (midpoint)
#print(T0.interp(T1, 0.5)) #from 0 to 1, normalized
Ts= T0.interp(T1, 50) #Store the trajectory, 50 steps
Ts.animate().save('Cartesian.gif')

P = Ts.t #Extract the translation
rtb.xplot(P, labels="x y z", block=True)
rpy = Ts.rpy(); #Extract the orientation
rtb.xplot(rpy, labels="roll pitch yaw", block=True)

Ts_2 = rtb.ctraj(T0, T1, 50)
# Ts = T0.interp(T1, rtb.trapezoidal(0, 1, 50).q) #Equivalent to the above
Ts_2.animate(limits=[-0.8, 0.8, -0.8, 0.8, -1, 1]).save('Cartesian2.gif')
P_2 = Ts_2.t #Extract the translation
rtb.xplot(P_2, labels="x y z", block=True)
rpy_2 = Ts_2.rpy(); #Extract the orientation
rtb.xplot(rpy_2, labels="roll pitch yaw", block=True)
