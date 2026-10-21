import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from spatialmath.base import rpy2tr
np.set_printoptions(suppress=True, precision=4)

robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(155)]),
        rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-155), np.deg2rad(90)]),
        rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi/2, qlim=[np.deg2rad(-80), np.deg2rad(200)]),
        rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi/2, qlim=[np.deg2rad(-165), np.deg2rad(165)]),
        rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, qlim=[np.deg2rad(-140), np.deg2rad(140)]),
        rtb.RevoluteDH(d=0.115, a=0.0, alpha=0, qlim=[np.deg2rad(-360), np.deg2rad(360)]),
    ], name="Kuka KR5", base=SE3(0,0,0))
print(robot)

robot.qz=[np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]
# robot.teach(robot.qz)

T_initial = robot.fkine(robot.qz)
print(f"Initial position matrix:\n{T_initial}")







#Roll 180 about X, Pitch 0.142 about Y, Yaw 0 about Z
T_target = SE3.Trans(0.848, -0.0, 0.28) * SE3(rpy2tr(180.0, 0.142, 0.0, 'deg'))
print("T_target matrix:\n", T_target, "\n")

#To keep the elbow up
q_initial = np.array([0, np.deg2rad(45), np.deg2rad(90), 0, np.deg2rad(0), 0])
robot.plot(q_initial, block=True)

q = robot.ikine_LM(T_target,q0=q_initial, ilimit=100, slimit=100, tol=1e-6)
print(f"Success: {q.success}")

if q.success:
    print(f"Inverse kinematics solution: {q.q}")
    T_result = robot.fkine(q.q)
    print(f"Resulting matrix after IK:\n{T_result}")
    # Show the robot in the computed position
    robot.plot(q.q, block=True)
