import roboticstoolbox as rtb
import spatialmath.base as smb
import spatialmath as sm
import numpy as np
np.set_printoptions(suppress=True, precision=4)

# Create a Puma560 instance
p560 = rtb.models.DH.Puma560()

# Plot the robot in its initial position (qz)
p560.plot(p560.qz, block=False)

# Define a homogeneous matrix (a translation and a rotation)
T = sm.SE3.Trans(0.4, 0.1, 0) * sm.SE3(smb.rpy2tr(0, 180, 0, 'deg'))
print(T, "\n")

# Plot the matrix (over the robot)
smb.trplot(T.A, block=True)

#Compute the inverse kinematics for the transformation matrix T
q = p560.ikine_a(T) #Default: left, elbow up
print(f"Left, elbow up\n {q.q}")
#In degrees
print(f"Left, elbow up, degrees:\n {np.rad2deg(q.q)}")
#Show the robot in the computed position
p560.plot(q.q, block=True)

#Compute the inverse with right configuration, elbow up
q = p560.ikine_a(T, config = 'ru')
print(f"Right, elbow up\n {q.q}")
#In degrees
print(f"Right, elbow up, degrees:\n {np.rad2deg(q.q)}")
p560.plot(q.q, block=True)

#Compute the inverse with right configuration, elbow down
q = p560.ikine_a(T, config = 'rd')
print(f"Right, elbow down\n {q.q}")
#In degrees
print(f"Right, elbow down, degrees:\n {np.rad2deg(q.q)}")
p560.plot(q.q, block=True)

#Compute the inverse with left configuration, elbow down
q = p560.ikine_a(T, config = 'ld')
print(f"Left, elbow down\n {q.q}")
#In degrees
print(f"Left, elbow down, degrees:\n {np.rad2deg(q.q)}")
p560.plot(q.q, block=True)
