import sympy as sp #To print the matrix
# To be able to plot
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(
    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})
#To use DH
import roboticstoolbox as rtb

traj = rtb.quintic(0, 1, np.linspace(0, 1, 50))
#Initial, final, time (from 0 to 1, with 50 steps)
traj.plot(block=True)

print(f"Position:\n")
sp.pprint(traj.q) #Position traj.q
print(f"Velocity:\n")
sp.pprint(traj.qd)#Velocity
print(f"Acceleration:\n") #Acceleration
sp.pprint(traj.qdd)

traj2 = rtb.quintic(0, 1, np.linspace(0, 1, 50), qd0=10, qdf=0)
#Initial and final pos, time (from 0 to 1, with 50 steps), with initial vel 10 and final 0
traj2.plot(block=True)

#Trapezoidal: Initial and final pos, time (from 0 to 1, with 50 steps)
traj3 = rtb.trapezoidal(0, 1, np.linspace(0, 1, 50))
traj3.plot(block=True)
#Trapezoidal: now defining max vel
traj3_2 = rtb.trapezoidal(0, 1, np.linspace(0, 1, 50), V=1.8)
traj3_2.plot(block=True)

traj2D =  rtb.mtraj( rtb.trapezoidal, [0, 2], [1, -2], 50)
#Initial coord (0,2) final (1,-2), with 50 steps
traj2D.plot(block=True)
traj2D_2 =  rtb.mtraj( rtb.quintic, [0, 2], [1, -2], 50)
traj2D_2.plot(block=True)