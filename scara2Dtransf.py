import numpy as np
import matplotlib.pyplot as plt
from spatialmath import *
from spatialmath.base import *

T0 = transl2(0,0) #Referencia
trplot2(T0, frame="0", color="k")

#Rotación de seguida de traslación, respecto a T0
TA = trot2(45, "deg")
trplot2(TA, frame="A", color="b")
plot_circle(4, (0, 0), 'b--')
#Para que la transformacion sea respecto a TA
TBA  = TA @ transl2(4, 0) @ trot2(60, "deg") 
trplot2(TBA , frame="B", color="g")
origin_TBA = TBA [:2, 2]
plot_circle(3, (origin_TBA[0], origin_TBA[1]), 'g--')

TCBA = TBA @ transl2(3,0)
trplot2(TCBA, frame="C", color="y")
print(TCBA)

origin_TCBA = TCBA[:2, 2]
P = np.array([origin_TCBA[0], origin_TCBA[1]])
plot_point(P, "ko", text="P")
print("Coordenadas en T0: {:.4f}, {:.4f}".format(P[0], P[1]))

P_TA = homtrans(np.linalg.inv(TA), P)
print("Coordenadas en TA: {:.4f}, {:.4f}".format(P_TA[0, 0], P_TA[1, 0]))
P_TBA = homtrans(np.linalg.inv(TBA), P)
print("Coordenadas en TBA: {:.4f}, {:.4f}".format(P_TBA[0, 0], P_TBA[1, 0]))

plt.axis('equal') 
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Brazo SCARA 2D')
plt.show() # Mostrar ventana 
