import sympy as sp
from sympy.matrices import rot_axis3
from sympy import Float
from sympy import preorder_traversal
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb

# Configurar NumPy para suprimir la notación científica y limitar la precisión
np.set_printoptions(suppress=True, precision=4,
                    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

# Esto fue generado para poder visualizar mejor las matrices
# Definir un umbral más pequeño para mostrar resultados
threshold = 1e-15  # Ajustado a un valor más pequeño
#Reemplaza los Float de SymPy menores que el umbral con cero
def replace_small_floats(expr, threshold):
    replacements = {}
    for term in preorder_traversal(expr):
        if isinstance(term, Float) and abs(term) < threshold:
            replacements[term] = Float(0.0)
    return expr.xreplace(replacements)
# Función para imprimir transformaciones y manejar valores pequeños
def print_transformation(T_i, name):
    print(f"\n{name}:")
    threshold = 1e-15  # Umbral para valores pequeños

    # Convertir a matriz SymPy si es de SE3
    if isinstance(T_i, SE3):
        matrix = sp.Matrix(T_i.A)
    else:
        matrix = sp.Matrix(T_i)

    matrix = sp.simplify(matrix)
    rows, cols = matrix.shape
    clean_matrix = sp.zeros(rows, cols)

    for i in range(rows):
        for j in range(cols):
            value = matrix[i, j]
            simplified_value = sp.simplify(value)

            # Reemplazar Floats pequeños y simplificar
            simplified_value = replace_small_floats(simplified_value, threshold)
            simplified_value = sp.simplify(simplified_value)

            # Evaluar si es numérico después del reemplazo
            if simplified_value.is_number:
                numeric_value = simplified_value.evalf()
                if abs(numeric_value) < threshold:
                    clean_matrix[i, j] = 0
                else:
                    clean_matrix[i, j] = sp.Float(numeric_value, 10)  # Redondear a 10 decimales
            else:
                # Intentar simplificar y verificar si es cero
                simplified_value = sp.simplify(simplified_value)
                if simplified_value.is_zero:
                    clean_matrix[i, j] = 0
                else:
                    clean_matrix[i, j] = simplified_value

    # Imprimir la matriz limpia
    sp.pprint(clean_matrix, use_unicode=True)
#Fin de lo generado

# Definir longitudes de eslabones como símbolos
L1, L2 = sp.symbols('L1 L2')

# Crear el robot con parámetros DH
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=L1, a=0, alpha=np.deg2rad(90), qlim=[-np.deg2rad(180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0, a=L2, alpha=0, qlim=[-np.deg2rad(180), np.deg2rad(180)]),
    ], name="EjemploBot", base=SE3(0, 0, 0))

print(robot)

# Joints simbólicas
q1, q2 = sp.symbols('q1 q2')

# Obtener todas las transformaciones
T = robot.fkine_all([q1, q2])

# Mostrar cada submatriz
for i, T_i in enumerate(T):
    print_transformation(T_i, f'Transformación para el eslabón {i}')

# Transformación final
Tfinal = robot.fkine([q1, q2])
print_transformation(Tfinal, "Transformación final")

# Jacobiano analítico (roll pitch yaw)
J = robot.jacob0_analytical([q1, q2])
print_transformation(J, "Jacobiano analítico")
print("\n")
#Cuando no sean simbólicas, usaremos robot.jacob0(q)

#Por ejemplo L1=0.4 y L2=0.3, y theta1=30 y theta2=45, el jacobiano:
robotNum = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0, alpha=np.deg2rad(90), qlim=[-np.deg2rad(180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0, a=0.3, alpha=0, qlim=[-np.deg2rad(180), np.deg2rad(180)]),
    ], name="EjemploBot", base=SE3(0, 0, 0))
q1=30
q2=45
Jn = robotNum.jacob0([np.deg2rad(q1), np.deg2rad(q2)])
print_transformation(Jn, "Jacobiano numérico")
