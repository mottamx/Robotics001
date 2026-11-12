import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
from swift import Swift

env = Swift()
env.launch(realtime=True)
dt = 0.05

panda = rtb.models.Panda()
panda.q = panda.qr
env.add(panda)

SIDE = 0.05
target_orientation = sm.SE3.Rx(np.pi)  # la pinza apunta hacia abajo

# Secuencia: (x, y, z, tiempo_de_movimiento_s, pausa_s). z es la altura sobre el piso.
WAYPOINTS = [
    (0.50,  0.00, 0.30, 0.0, 1.0),  # inicio
    (0.45, -0.25, 0.30, 2.5, 0.5),  # sobre A
    (0.45, -0.25, 0.08, 1.5, 1.0),  # baja a A
    (0.45, -0.25, 0.30, 1.5, 0.5),  # sube
    (0.45,  0.25, 0.30, 3.0, 0.5),  # sobre B
    (0.45,  0.25, 0.08, 1.5, 1.0),  # baja a B
    (0.45,  0.25, 0.30, 1.5, 0.5),  # sube
    (0.50,  0.00, 0.30, 2.5, 1.0),  # regresa al inicio
]
WARMUP = 3.0  # el robot llega primero al cubo antes de iniciar la secuencia

# Línea de tiempo: [(t_inicio, t_fin_mov, t_fin_pausa, p_ini, p_fin), ...]
segments, t_acc = [], 0.0
for i in range(1, len(WAYPOINTS)):
    p0 = np.array(WAYPOINTS[i - 1][:3])
    x, y, z, t_move, t_pause = WAYPOINTS[i]
    segments.append((t_acc, t_acc + t_move, t_acc + t_move + t_pause, p0, np.array([x, y, z])))
    t_acc += t_move + t_pause
TOTAL = t_acc


def smooth(u):
    """Interpolación suave (mínimo jerk): arranca y frena con velocidad cero."""
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def target_position(t):
    t = max(t - WARMUP, 0.0) % TOTAL  # se repite en bucle
    for t0, t1, t2, p0, p1 in segments:
        if t < t2:
            u = 1.0 if t >= t1 else (t - t0) / (t1 - t0)
            return p0 + (p1 - p0) * smooth(u)
    return segments[-1][4]


x0, y0, z0 = WAYPOINTS[0][:3]
target = sg.Cuboid(
    [SIDE, SIDE, SIDE],
    pose=sm.SE3(x0, y0, z0 + SIDE / 2) * target_orientation,
    color=[0.2, 0.4, 1.0, 1.0],
)
env.add(target)

t = 0.0
while True:
    x, y, z = target_position(t)
    target.T = sm.SE3(x, y, z + SIDE / 2) * target_orientation

    v, _ = rtb.p_servo(panda.fkine(panda.q), target.T, gain=1.0, threshold=0.01)
    panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v

    env.step(dt)
    t += dt