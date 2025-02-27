#Trayectorias (con puntos cartesianos) y cinemática inversa
#Versión mejorada

import sys
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3
np.set_printoptions(precision=2)

#print(sys.version_info) #Verificar que version de python ejecuta esto

#Declaramos nuestro robot, incluidos limites de movimiento
robot= rtb.DHRobot([
    rtb.RevoluteDH(d=0.4, a=.18, alpha=np.pi/2, qlim=[-2.705, 2.705]),
    rtb.RevoluteDH(d=0, a=0.6, alpha=0, offset=np.pi/2, qlim=[-np.pi, 1.134]),
    rtb.RevoluteDH(d=0, a=0.12, alpha=np.pi/2, qlim=[-0.2618, 2.757]),
    rtb.RevoluteDH(d=0.62, a=0, alpha=-np.pi/2, qlim=[-6.10, 6.10]),
    rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[-2.268, 2.268]),
    rtb.RevoluteDH(d=0.05, a=0, alpha=0, qlim=[-6.10, 6.10]),
    ], name="Robot KR5", base=SE3([0, 0, 0]))

#**********Esto es nuevo 1 (inicio)
#Ponemos el TCP viendo hacia "abajo" y evitamos codo abajo
robot.tool=SE3.RPY(180.0, 0.0, -180.0, order='xyz', unit='deg')
robot.configurations_str('ru') #Right, elbow Up
#**********Esto es nuevo 1 (fin)

#Para verificar que quedo igual el DH
print(robot)

#***Cinematica inversa***
#Declaramos unos puntos en el espacio (CARTESIANO)
#Verificar que esten dentro de su workspace (con la datasheet)

#Puntos en el espacio, X, Y, Z
'''
T=([SE3([-0.5, 0.5, 0.85]),
    SE3([-0.5, 0.5, 0.65]),
    SE3([-0.5, -0.5, 0.65]),
    SE3([-0.5, -0.5, 0.85]),
    SE3([0.5, -0.5, 0.85]),
    SE3([0.5, -0.5, 0.65]),
    
    SE3([0.5, 0.5, 0.65]),
    SE3([0.5, 0.5, 0.85]),
    SE3([-0.5, 0.5, 0.85])])
'''
T=([SE3([-0.5, 0.5, 0.85]),
    SE3([-0.5, 0.5, 0.65]),
    SE3([-0.5, -0.5, 0.65])])
#**********Esto es nuevo 2 (inicio)**********
#Elegir algoritmo de cinematica inversa
#1 para minimizar, 2 para Lavenberg-Marquad-Sugihara
algoritmo=2

q=np.empty([0,6])#Puntos trayectoria, 6 articulares
for index, x in enumerate(T[:-1]): #Recorrer todos los puntos menos ultimo
    #print(T[index])
    #Regresa trayectoria geometrica de i a i-1
    #Con n pasos intermedios (el ultimo valor son los pasos) **Los pasos se pueden modificar
    qt = rtb.tools.trajectory.ctraj(T[index], T[index+1], 100) 
    
    if(algoritmo==1):
        #Opcion 1
        #Calculamos la cinematica inversa para toda la trayectoria
        Q=robot.ikine_min(T=qt, qlim=True, method='trust-constr', ilimit=6000) #mas ilim lento
        print(Q) #Para verificar que diga True
        #Extraer solo los valores de las joints de la inversa
        Q=list(reversed(Q)) #Para que no cambie el orden de los puntos
        print("Extrayendo solo valores art. \n Paso:", end=" ")
        print(index)
        for index2, x in enumerate(Q):
            #print(Q[index2].q) #Si queremos verificar que extrae
            q=np.insert(q, index2, [Q[index2].q], 0)   
        print("Exito")
    elif (algoritmo==2):
        #Opcion2
        #Calculamos la cinematica inversa para toda la trayectoria
        Q=robot.ikine_LMS(T=qt, ilimit=6000)
        print(Q) #Para verificar que diga True
        pre_q=Q.q[:].tolist() #Cambiar tipo de dato (pot Toolbox)
        #print(pre_q)
        pre_q=list(reversed(pre_q))  #Para que no cambie el orden de los puntos
        #Extraer solo los valores de las joints de la inversa
        print("Extrayendo solo valores art. \n Paso:", end=" ")
        print(index)
        for index2, x in enumerate(pre_q):
            q=np.insert(q, index2, [pre_q[index2]], 0)   
        print("Exito")

#Para visualizar todas las posiciones articulares
print("Variables articulares por las que pasa TCP")
q=np.flipud(q) #Para que lea los puntos en orden original
#np.set_printoptions(precision=2) #Para evitar notacion cientifica
print(q) #Para revisar si están en orden
#**********Esto es nuevo 2 (fin)

#Graficamos posiciones art obtenidas de inversa
#dt es segundos
#Con limits hacemos que el plot3d tenga tamaño [-x x -y y -z z]
robot.plot(q=qt.q, backend='pyplot', dt=0.025, name=True, movie="robot.gif", block=True, limits=[-1, 1, -1, 1, 0, 2], shadow=True) #Ya no lleva loop