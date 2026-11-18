import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SO2

fig, ax = plt.subplots()
via = SO2(30, unit="deg") * np.array([[-1, 1, 1, -1, -1], [1, 1, -1, -1, 1]])
x = via[0]
y = via[1]
ax.scatter(x, y, color='black')

traj0 = rtb.mstraj(via.T, dt=0.2, tacc=0, qdmax=[2, 1])
traj1 = rtb.mstraj(via.T, dt=0.2, tacc=0.5, qdmax=[2, 1])
traj2 = rtb.mstraj(via.T, dt=0.2, tacc=2, qdmax=[2, 1])

ax.plot(traj0.q[:, 0], traj0.q[:, 1], color="red", label="t_acc=0")
ax.plot(traj1.q[:, 0], traj1.q[:, 1], color="blue", label="t_acc=0.5")
ax.plot(traj2.q[:, 0], traj2.q[:, 1], color="green", label="t_acc=2")

plt.legend()
plt.show()
