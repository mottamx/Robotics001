import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import *
from spatialmath.base import *

# Parámetros del mandala
n_brazos = 8  # Número de brazos radiales
segmentos_por_brazo = 4  # Segmentos en cada brazo
L = 0.8  # Longitud de cada segmento
angulo_segmento = 15  # Ángulo entre segmentos del mismo brazo (grados)

# Frame de referencia central
T0 = transl2(0, 0)
trplot2(T0, frame="0", color="k", width=2)

# Colores para los brazos
colores = ['b', 'r', 'g', 'm', 'c', 'orange', 'purple', 'brown']

# Crear cada brazo radial
angulo_entre_brazos = 360 / n_brazos

for brazo in range(n_brazos):
    # Rotación inicial del brazo desde el centro
    T_brazo = T0 @ trot2(brazo * angulo_entre_brazos, "deg")

    color_brazo = colores[brazo % len(colores)]

    # Crear segmentos del brazo
    for seg in range(segmentos_por_brazo):
        # Trasladar y rotar para el siguiente segmento
        T_brazo = T_brazo @ transl2(L, 0) @ trot2(angulo_segmento, "deg")

        # Visualizar el frame
        frame_name = f"B{brazo}S{seg+1}"
        trplot2(T_brazo, frame=frame_name, color=color_brazo, width=0.5, length=0.3)

plt.axis('equal')
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.title(f'Mandala de {n_brazos} brazos con {segmentos_por_brazo} segmentos')
plt.tight_layout()
plt.show()