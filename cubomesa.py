# Imports básicos
import numpy as np
import matplotlib.pyplot as plt
# Robotics toolbox imports
import roboticstoolbox as rtb
# Spatialmath imports
from spatialmath import SE3
from spatialmath.base import trprint
# Sympy imports
import sympy as sp
from sympy import symbols, Float, preorder_traversal
from sympy.matrices import rot_axis3

# Configurar NumPy para suprimir la notación científica y limitar la precisión
np.set_printoptions(suppress=True, precision=4,
                    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})
threshold = 1e-15
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


# Declaramos nuestro robot, incluidos limites de movimiento
robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(d=1.5, a=0, alpha=np.pi/2, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0, a=2, alpha=0, offset=np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0, a=1, alpha=0, offset=-np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
    ], name="FakeRobot", base=SE3(0,0,0))

#Primero simbolicas para sacar las matrices intermedias
q1, q2, q3 = symbols('q1 q2 q3')
q = [q1, q2, q3]
print("Transformaciones individuales entre articulaciones:")
for i in range(len(robot.links)):
    # Usamos el método A de cada eslabón directamente
    A_i = robot.links[i].A(q[i])
    print(f"\nTransformación del eslabón {i} al {i+1} (usando q{i+1}):")
    print(A_i)

#Con valores
individual_transforms = []
q_values = [np.deg2rad(40), np.deg2rad(-50), 0]
print("\nTransformaciones individuales con valores numéricos:")
for i in range(len(robot.links)):
    A_i = robot.links[i].A(q_values[i])
    individual_transforms.append(A_i)
    # Mostrar el valor en grados
    print(f"\nTransformación del eslabón {i} al {i+1} (q{i+1}={np.rad2deg(q_values[i]):.1f}° = {q_values[i]:.2f} rad):")
    print(A_i)

# Multiplicar las transformaciones individuales
if isinstance(robot.base, SE3):
    T_multiplicada = robot.base
else:
    T_multiplicada = SE3()  # Matriz identidad si base no es un SE3 válido

for A in individual_transforms:
    if not isinstance(A, SE3):
        A = SE3(A)  # Convertir a SE3 si no lo es
    T_multiplicada = T_multiplicada @ A
print("\nTransformación final mediante multiplicación de transformaciones individuales:")
print(T_multiplicada)

# Obtener la transformación final directamente
T_final = robot.fkine(q_values)
print("\nTransformación final calculada directamente con fkine:")
print(T_final)

#Obtener la inversa de T_final
T_inv=T_final.inv()
print("\nTransformación inversa:")
print(T_inv)

#Cubo respecto a la mesa
Tcm=SE3.Trans([-0.4,0.4,0])
print("\nTransformación del cubo respecto a la mesa:")
print(Tcm)

#Mesa respecto a la base
Tmb=SE3.Trans([0,1,1])
print("\nTransformación de la mesa respecto a la base:")
print(Tmb)

# Calcular T_d^2 = (T_0^d)^{-1} · T_0^{mesa} · T_{mesa}^{cubo}
T_d2 = T_inv * Tmb * Tcm
print("\nTransformación final T_d^2:")
print(T_d2)