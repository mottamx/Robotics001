import sympy as sp
from sympy.matrices import rot_axis3
from sympy import Float
from sympy import preorder_traversal
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb

# Configure NumPy to suppress scientific notation and limit precision
np.set_printoptions(suppress=True, precision=4,
                    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

# The following was generated to make the matrices easier to read
# Define a smaller threshold to display results
threshold = 1e-15  # Adjusted to a smaller value
# Replace SymPy Floats smaller than the threshold with zero
def replace_small_floats(expr, threshold):
    replacements = {}
    for term in preorder_traversal(expr):
        if isinstance(term, Float) and abs(term) < threshold:
            replacements[term] = Float(0.0)
    return expr.xreplace(replacements)
# Function to print transformations and handle small values
def print_transformation(T_i, name):
    print(f"\n{name}:")
    threshold = 1e-15  # Threshold for small values

    # Convert to a SymPy matrix if it is an SE3
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

            # Replace small Floats and simplify
            simplified_value = replace_small_floats(simplified_value, threshold)
            simplified_value = sp.simplify(simplified_value)

            # Check whether it is numeric after the replacement
            if simplified_value.is_number:
                numeric_value = simplified_value.evalf()
                if abs(numeric_value) < threshold:
                    clean_matrix[i, j] = 0
                else:
                    clean_matrix[i, j] = sp.Float(numeric_value, 10)  # Round to 10 digits
            else:
                # Try to simplify and check whether it is zero
                simplified_value = sp.simplify(simplified_value)
                if simplified_value.is_zero:
                    clean_matrix[i, j] = 0
                else:
                    clean_matrix[i, j] = simplified_value

    # Print the clean matrix
    sp.pprint(clean_matrix, use_unicode=True)
# End of the generated section

# Define the link lengths as symbols
L1, L2 = sp.symbols('L1 L2')

# Create the robot with DH parameters
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=L1, a=0, alpha=np.deg2rad(90), qlim=[-np.deg2rad(180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0, a=L2, alpha=0, qlim=[-np.deg2rad(180), np.deg2rad(180)]),
    ], name="ExampleBot", base=SE3(0, 0, 0))

print(robot)

# Symbolic joint variables
q1, q2 = sp.symbols('q1 q2')

# Get all the transformations
T = robot.fkine_all([q1, q2])

# Show each sub-matrix
for i, T_i in enumerate(T):
    print_transformation(T_i, f'Transformation for link {i}')

# Final transformation
Tfinal = robot.fkine([q1, q2])
print_transformation(Tfinal, "Final transformation")

# Geometric Jacobian (symbolic)
J = robot.jacob0([q1, q2])
print_transformation(J, "Geometric Jacobian")
print("\n")
# With numeric joint values the call is the same: robot.jacob0(q)

# For example, with L1=0.4, L2=0.3, theta1=30 and theta2=45, the Jacobian is:
robotNum = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.4, a=0, alpha=np.deg2rad(90), qlim=[-np.deg2rad(180), np.deg2rad(180)]),
        rtb.RevoluteDH(d=0, a=0.3, alpha=0, qlim=[-np.deg2rad(180), np.deg2rad(180)]),
    ], name="ExampleBot", base=SE3(0, 0, 0))
q1=30
q2=45
Jn = robotNum.jacob0([np.deg2rad(q1), np.deg2rad(q2)])
print_transformation(Jn, "Geometric Jacobian (numeric)")
