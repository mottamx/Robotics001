import roboticstoolbox as rtb
import os

# Obtener la ruta del archivo JSON
file_path = rtb.rtb_path_to_datafile("data/hershey.json")

print("Ruta del archivo:", file_path)

# Verificar si el archivo existe
print("¿Existe el archivo?", os.path.exists(file_path))
