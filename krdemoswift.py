"""
krdemoswift.py
Demo en Swift de un KUKA KR5 declarado con parámetros DH (sin URDF ni mallas).

Como un DHRobot no trae geometría, cada eslabón se dibuja a mano con cilindros
y esferas de spatialgeometry, y su pose se actualiza en cada paso a partir de
robot.fkine_all(q). El robot NO se agrega a Swift: solo se agregan las formas.

Flujo:
  1. Declara el KR5 (igual que en kr_cube.py).
  2. Genera la trayectoria cartesiana del cubo con ctraj.
  3. Resuelve la cinemática inversa (ikine_LM) para cada pose.
  4. Reproduce qtraj en Swift en bucle, listo para grabar.
"""
import itertools

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
from spatialmath import SE3
from swift import Swift

# ----------------------------------------------------------------------------
# Parámetros de la demo (ajusta aquí)
# ----------------------------------------------------------------------------
STEPS_PER_EDGE = 30   # pasos de ctraj por arista (más = más fluido, más IK)
DT = 0.05             # tiempo entre cuadros del visor, en segundos
PAUSE_BEFORE = True   # esperar Enter antes de animar (para iniciar la grabación)

# Cubo: punto pivote C y tamaño de la arista (los mismos de kr_cube.py)
X_C, Y_C, Z_C = -0.6, 0.6, 1.3
VERTICE = 1.30

LINK_RADIUS = 0.04
LINK_COLOR = [0.95, 0.45, 0.10, 1.0]   # naranja KUKA
JOINT_COLOR = [0.25, 0.25, 0.25, 1.0]
CUBE_COLOR = [0.2, 0.4, 1.0, 1.0]
TCP_COLOR = [1.0, 0.1, 0.1, 1.0]


# ----------------------------------------------------------------------------
# 1. Robot
# ----------------------------------------------------------------------------
def build_robot():
    robot = rtb.DHRobot(
        [
            rtb.RevoluteDH(d=0.4, a=0.180, alpha=np.pi / 2,
                           qlim=[np.deg2rad(-155), np.deg2rad(155)]),
            rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi / 2,
                           qlim=[np.deg2rad(-180), np.deg2rad(65)]),
            rtb.RevoluteDH(d=0, a=0.120, alpha=np.pi / 2,
                           qlim=[np.deg2rad(-110), np.deg2rad(170)]),
            rtb.RevoluteDH(d=0.620, a=0.0, alpha=-np.pi / 2,
                           qlim=[np.deg2rad(-165), np.deg2rad(165)]),
            rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi / 2,
                           qlim=[np.deg2rad(-140), np.deg2rad(140)]),
            rtb.RevoluteDH(d=0.115, a=0.0, alpha=0,
                           qlim=[np.deg2rad(-360), np.deg2rad(360)]),
        ],
        name="Kuka KR5",
        base=SE3(0, 0, 0),
    )

    # TCP alineado con el brazo
    robot.tool = SE3.OA([0, 1, 0], [0, 0, 1])
    robot.configurations_str('ru')  # Right, elbow Up
    robot.qz = np.zeros(6)
    robot.qhome = np.deg2rad([0.0, 45.0, 90.0, 0.0, 45.0, 0.0])
    return robot


# ----------------------------------------------------------------------------
# 2. Trayectoria del cubo
# ----------------------------------------------------------------------------
def cube_vertices():
    v = VERTICE
    return {
        "A": (X_C,     Y_C - v, Z_C - v),
        "B": (X_C,     Y_C - v, Z_C),
        "C": (X_C,     Y_C,     Z_C),
        "D": (X_C,     Y_C,     Z_C - v),
        "E": (X_C + v, Y_C,     Z_C - v),
        "F": (X_C + v, Y_C,     Z_C),
        "G": (X_C + v, Y_C - v, Z_C),
        "H": (X_C + v, Y_C - v, Z_C - v),
    }


def vertex_pose(xyz):
    """Misma orientación que en kr_cube.py: arriba RPY(0,0,-46), abajo RPY(180,36,-81)."""
    x, y, z = xyz
    if abs(z - Z_C) < 1e-9:
        return SE3(x, y, z) * SE3.RPY(0, 0, -46, unit='deg')
    return SE3(x, y, z) * SE3.RPY(180, 36, -81, unit='deg')


def build_waypoints():
    V = cube_vertices()
    order = "ABCDAHGFEHGBCFED"  # recorrido de kr_cube.py
    return [vertex_pose(V[k]) for k in order]


def ik(robot, pose, q0, tries=40):
    """ikine_LM desde q0; si falla, reintenta con semillas aleatorias dentro de qlim."""
    sol = robot.ikine_LM(pose, q0=q0, tol=1e-5, ilimit=500, slimit=25)
    if sol.success:
        return sol.q
    rng = np.random.default_rng(0)
    lo, hi = robot.qlim
    for _ in range(tries):
        sol = robot.ikine_LM(pose, q0=rng.uniform(lo, hi), tol=1e-5, ilimit=500, slimit=10)
        if sol.success and np.all(sol.q >= lo - 1e-6) and np.all(sol.q <= hi + 1e-6):
            return sol.q
    return None


def solve_edge(robot, T_start, T_end, q_start):
    """Arista en línea recta (ctraj + IK). Si algún punto intermedio no es alcanzable,
    interpola en espacio articular entre las soluciones de los dos vértices.
    Devuelve (lista de q, True si fue recta)."""
    qs, q0 = [], q_start
    for pose in rtb.ctraj(T_start, T_end, STEPS_PER_EDGE):
        q = ik(robot, pose, q0, tries=0)
        if q is None:
            break
        qs.append(q)
        q0 = q
    else:
        return qs, True

    q_end = ik(robot, T_end, q_start)
    if q_end is None:
        raise ValueError(f"Vértice no alcanzable: {T_end.t}")
    return list(rtb.jtraj(q_start, q_end, STEPS_PER_EDGE).q), False


def solve_trajectory(robot, waypoints):
    print("Resolviendo cinemática inversa...")
    q_start = ik(robot, waypoints[0], robot.qz)
    if q_start is None:
        raise ValueError(f"Vértice inicial no alcanzable: {waypoints[0].t}")
    qtraj = [q_start]
    fallbacks = []
    for i in range(len(waypoints) - 1):
        print(f"\rArista {i + 1}/{len(waypoints) - 1}", end="")
        qs, straight = solve_edge(robot, waypoints[i], waypoints[i + 1], qtraj[-1])
        if not straight:
            fallbacks.append(i + 1)
        qtraj.extend(qs)
    print()
    if fallbacks:
        print(f"Aviso: las aristas {fallbacks} no se pueden recorrer en línea recta "
              "con esa orientación (fuera de alcance); se interpolaron en espacio articular.")
    return np.array(qtraj)


# ----------------------------------------------------------------------------
# 3. Dibujo del robot con formas de spatialgeometry
# ----------------------------------------------------------------------------
def segment_pose(p0, p1):
    """Pose de un cilindro (eje z local) que va de p0 a p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    mid = (p0 + p1) / 2
    a = p1 - p0
    a = a / np.linalg.norm(a)
    o = [1, 0, 0] if abs(a[0]) < 0.9 else [0, 1, 0]
    return SE3(mid[0], mid[1], mid[2]) * SE3.OA(o, a)


class RobotDrawing:
    """Dibuja un DHRobot como cilindros (d y a de cada eslabón) y esferas (articulaciones)."""

    def __init__(self, env, robot, radius=LINK_RADIUS):
        self.robot = robot
        self.parts = []   # (link_index, tipo, shape, longitud)
        self.joints = []  # esferas en los ejes de articulación
        for i, link in enumerate(robot.links):
            if abs(link.d) > 1e-6:
                cyl = sg.Cylinder(radius=radius, length=abs(link.d), color=LINK_COLOR)
                env.add(cyl)
                self.parts.append((i, "d", cyl))
            if abs(link.a) > 1e-6:
                cyl = sg.Cylinder(radius=radius, length=abs(link.a), color=LINK_COLOR)
                env.add(cyl)
                self.parts.append((i, "a", cyl))
            ball = sg.Sphere(radius=radius * 1.35, color=JOINT_COLOR)
            env.add(ball)
            self.joints.append(ball)
        self.tcp = sg.Sphere(radius=radius * 1.2, color=TCP_COLOR)
        env.add(self.tcp)

    def update(self, q):
        Ts = self.robot.fkine_all(q)  # Ts[0] = base, Ts[i] = marco del eslabón i
        for i, ball in enumerate(self.joints):
            ball.T = SE3(Ts[i].t)
        for i, kind, cyl in self.parts:
            T_prev, T_i = Ts[i], Ts[i + 1]
            p_prev = T_prev.t
            p_mid = p_prev + self.robot.links[i].d * T_prev.R[:, 2]
            if kind == "d":
                cyl.T = segment_pose(p_prev, p_mid)
            else:
                cyl.T = segment_pose(p_mid, T_i.t)
        self.tcp.T = SE3(self.robot.fkine(q).t)


def add_cube_wireframe(env, radius=0.012):
    V = cube_vertices()
    names = list(V)
    for n1, n2 in itertools.combinations(names, 2):
        p1, p2 = np.array(V[n1]), np.array(V[n2])
        diff = np.abs(p1 - p2) > 1e-9
        if diff.sum() == 1:  # vértices que difieren en un solo eje = arista
            cyl = sg.Cylinder(radius=radius, length=float(np.linalg.norm(p1 - p2)),
                              color=CUBE_COLOR)
            cyl.T = segment_pose(p1, p2)
            env.add(cyl)
    for p in V.values():
        s = sg.Sphere(radius=radius * 2, color=CUBE_COLOR)
        s.T = SE3(*p)
        env.add(s)


# ----------------------------------------------------------------------------
# 4. Main
# ----------------------------------------------------------------------------
def main():
    robot = build_robot()
    print(robot)

    waypoints = build_waypoints()
    qtraj = solve_trajectory(robot, waypoints)

    env = Swift()
    env.launch(realtime=True)

    add_cube_wireframe(env)
    drawing = RobotDrawing(env, robot)
    drawing.update(qtraj[0])
    env.step(DT)

    if PAUSE_BEFORE:
        input("Acomoda la ventana, inicia la grabación y presiona Enter...")

    try:
        while True:
            for q in qtraj:
                drawing.update(q)
                env.step(DT)
    except KeyboardInterrupt:
        print("\nDetenido.")


if __name__ == "__main__":
    main()
