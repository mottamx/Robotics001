import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
from swift import Swift, Slider

env = Swift()
env.launch(realtime=True, browser="chrome")
dt = 0.05

panda = rtb.models.Panda()
panda.q = panda.qr
env.add(panda)

SIDE = 0.05
X0, Y0, Z0 = 0.5, 0.0, 0.3  # posición inicial del objetivo; z es la altura sobre el piso

# La pinza apunta hacia abajo
target_orientation = sm.SE3.Rx(np.pi)

target = sg.Cuboid(
    [SIDE, SIDE, SIDE],
    pose=sm.SE3(X0, Y0, Z0 + SIDE / 2) * target_orientation,
    color=[0.2, 0.4, 1.0, 1.0],
)
env.add(target)

# Los sliders escriben aquí; el loop principal lee de aquí
pos = {"x": X0, "y": Y0, "z": Z0}


def make_cb(key):
    def cb(value):
        pos[key] = float(value)
    return cb


env.add(Slider(make_cb("x"), min=0.2, max=0.7, step=0.01, value=X0, desc="Target X", unit="m"))
env.add(Slider(make_cb("y"), min=-0.4, max=0.4, step=0.01, value=Y0, desc="Target Y", unit="m"))
env.add(Slider(make_cb("z"), min=0.05, max=0.6, step=0.01, value=Z0, desc="Target Z", unit="m"))

while True:
    # Mover el cubo a la posición de los sliders
    target.T = sm.SE3(pos["x"], pos["y"], pos["z"] + SIDE / 2) * target_orientation

    # Control: servo de posición hacia el cubo
    v, _ = rtb.p_servo(panda.fkine(panda.q), target.T, gain=1.0, threshold=0.01)
    panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v

    env.step(dt)  # integra q += qd*dt y actualiza el visor