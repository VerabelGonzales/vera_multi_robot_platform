import matplotlib.pyplot as plt
import numpy as np

# Datos proporcionados por el usuario
distancias_reales = [
    10.00, 15.00, 20.00, 25.00, 30.00, 35.00, 40.00, 45.00,
    50.00, 55.00, 60.00, 65.00, 70.00, 75.00, 80.00
]

promedios_lecturas = [
9.77,
14.82,
20.01,
25.87,
31.72,
36.87,
41.62,
46.97,
53.01,
59.27,
65.42,
73.47,
83.32,
90.22,
102.37,
]

# Graficando la curva de respuesta del sensor con la curva de color azul
plt.figure(figsize=(10, 6))
plt.plot(distancias_reales, promedios_lecturas, 'o-', color='y', label='Promedio de Lecturas del Sensor IR 4')
plt.title('Curva de Respuesta del Sensor IR 4')
plt.xlabel('Distancia Real (cm)')
plt.ylabel('Promedio de Lectura del Sensor (cm)')
plt.grid(True)
plt.legend()
plt.show()


