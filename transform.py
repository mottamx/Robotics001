import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt

from spatialmath import *
from spatialmath.base import *

# from sympy import Symbol, Matrix

theta_deg = 30
theta_rad = np.deg2rad(theta_deg)

# R = rot2(theta_rad)
# print(R)

# R2=trot2(theta_rad)
# trplot2(R)

T0 = transl2(0,0) #Referencia
trplot2(T0, frame="0", color="k")

#Traslación de 1,2 seguida de rotación de 30 grados
TA = transl2(1,2) @ trot2(theta_deg, "deg")
print(TA)
trplot2(TA, frame="A", color="b")

#Rotación de 30 grados seguida de traslación de 1,2
# TB = trot2(theta_deg, "deg") @ transl2(1,2)
# print(TB)
# trplot2(TB, frame="B", color="r")

P=np.array([4,3])
plot_point(P, "ko", text="P")

P1=homtrans(np.linalg.inv(TA), P)
print(P1)

plt.axis('equal')
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Transformación 2D')
plt.show()