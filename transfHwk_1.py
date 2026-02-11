import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import *
from spatialmath.base import *

L = 1.5  # Longitud de cada lado
angulo = 60  # Ángulo entre segmentos (360/6 = 60°)

# Frame de referencia
T0 = transl2(0, 0)
trplot2(T0, frame="0", color="k")

# Frame 1: trasladar L en x, rotar 60°
T1 = T0 @ transl2(L, 0) @ trot2(angulo, "deg")
trplot2(T1, frame="1", color="b")

# Frame 2: desde T1, trasladar L, rotar 60°
T2 = T1 @ transl2(L, 0) @ trot2(angulo, "deg")
trplot2(T2, frame="2", color="r")

# Frame 3: desde T2, trasladar L, rotar 60°
T3 = T2 @ transl2(L, 0) @ trot2(angulo, "deg")
trplot2(T3, frame="3", color="g")

# Frame 4: desde T3, trasladar L, rotar 60°
T4 = T3 @ transl2(L, 0) @ trot2(angulo, "deg")
trplot2(T4, frame="4", color="m")

# Frame 5: desde T4, trasladar L, rotar 60°
T5 = T4 @ transl2(L, 0) @ trot2(angulo, "deg")
trplot2(T5, frame="5", color="c")

# Frame 6: desde T5, trasladar L, rotar 60° (cierra el hexágono)
T6 = T5 @ transl2(L, 0) @ trot2(angulo, "deg")
trplot2(T6, frame="6", color="y")

plt.axis('equal')
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('HexagonoTransformaciones')
plt.show()