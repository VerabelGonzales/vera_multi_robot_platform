import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Cargar los datos desde el archivo Excel
file_path = 'datos_robot_3.xlsx'
data = pd.read_excel(file_path)

# Convertir las series de Pandas a arrays de NumPy
tiempo = data['Tiempo [s]'].to_numpy()
set_point = data['Set point [RPM]'].to_numpy()
velocidad_m1 = data['Velocidad motor 1 [RPM]'].to_numpy()
velocidad_m2 = data['Velocidad motor 2 [RPM]'].to_numpy()
velocidad_m3 = data['Velocidad motor 3 [RPM]'].to_numpy()
velocidad_m4 = data['Velocidad motor 4 [RPM]'].to_numpy()

# Crear la gráfica
plt.figure(figsize=(12, 6))

# Graficar cada serie de datos con colores distintos y líneas más delgadas
plt.plot(tiempo, set_point, label='Set point [RPM]', color='black', linewidth=1)
plt.plot(tiempo, velocidad_m1, label='Velocidad motor 1 [RPM]', color='red', linewidth=1)
plt.plot(tiempo, velocidad_m2, label='Velocidad motor 2 [RPM]', color='blue', linewidth=1)
plt.plot(tiempo, velocidad_m3, label='Velocidad motor 3 [RPM]', color='green', linewidth=1)
plt.plot(tiempo, velocidad_m4, label='Velocidad motor 4 [RPM]', color='purple', linewidth=1)

# Añadir títulos y etiquetas
plt.title('Control de Velocidad del Robot 3')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad [RPM]')
plt.grid(True)
plt.legend()

# Mostrar la gráfica
plt.show()

