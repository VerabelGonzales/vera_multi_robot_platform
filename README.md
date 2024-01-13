# Plataforma robotica educativa para la implementacion de algoritmos relacionados a sistemas multi-robot mobiles
## Configuración del Workspace en ROS para la plataforma 
Para trabajar con el proyecto de robótica, necesitarás configurar un workspace en ROS y clonar este repositorio en tu máquina local. Sigue los pasos a continuación para configurarlo correctamente.

### Paso 1: Instalar ROS

Asegúrate de tener instalada la versión correcta de ROS en tu sistema. Para esta plataforma, se esta utilizando ROS Noetic, que es compatible con Ubuntu 20.04. Puedes encontrar las instrucciones de instalación en [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).

### Paso 2: Crear un Workspace en ROS

Abre una terminal y ejecuta los siguientes comandos para crear tu workspace de ROS:
```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/limo_ros.git
    $ cd ..
    $ catkin_make
```
### Paso 3: Configurar el Entorno ROS
Agrega el workspace a tu entorno ROS para que se cargue automáticamente cada vez que abras una terminal:
```
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc

```

### Paso 4: Clonar el Repositorio
Ahora, clona este repositorio dentro del subdirectorio src de tu workspace:
```
    cd ~/catkin_ws/src
    git clone [URL_DEL_REPOSITORIO]
```
Reemplaza [URL_DEL_REPOSITORIO] con la URL de este repositorio en GitHub.

### Paso 5: Compilar el Workspace
Después de clonar el repositorio, regresa al directorio raíz de tu workspace y ejecuta catkin_make para compilar el proyecto:
```
    cd ~/catkin_ws
    catkin_make

```

### Paso 6: Listo para Trabajar
¡Ya estás listo para comenzar a trabajar con la plataforma! Asegúrate de seguir cualquier instrucción adicional específica del repositorio para ejecutar o desarrollar el proyecto.
## Diseño del robot en SolidWorks
<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Acople_Motor_II.PNG" alt="Acoples">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 1.</strong> Acoples</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Base.PNG" alt="Base, vista superior">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 2.</strong> Base, vista superior</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Base_2.PNG" alt="Base isométrica, vista frontal, lateral">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 3.</strong> Base isométrica, vista frontal, lateral</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Mecanum%20Wheel.PNG" alt="Ruedas mecanum">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 4.</strong> Ruedas mecanum</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Tapa_2_2.PNG" alt="Tapa inferior del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 5.</strong> Tapa inferior del robot</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Tapa_3.PNG" alt="Tapa superior del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 6.</strong> Tapa superior del robot</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Tapa_Base.PNG" alt="Tapa base del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 7.</strong> Tapa base del robot</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Tapa_Frontal.PNG" alt="Tapa frontal del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 8.</strong> Tapa frontal del robot</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Tapa_I.PNG" alt="Tapa lateral del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 9.</strong> Tapa lateral del robot</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Partes_Robot_1.png" alt="Tapa frontal del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 10.</strong> Vista Isométrica del robot</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Partes_Robot_2.png" alt="Tapa frontal del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 11.</strong> Vista frontal del robot</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Partes_Robot_3.png" alt="Tapa frontal del robot">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 12.</strong> Vista lateral del robot</p>
<br>

## Número de componentes

| **Componente**            | **Unidades**|
|---------------------------|----------|
| Rueda Mecanum             | 4        |
| Motores                   | 4        |
| Puente H L298N            | 2        |
| Sensor IR                 | 4        |
| Pantalla OLED             | 1        |
| Indicador Nivel de Batería| 1        |
| Regulador XL4015          | 1        |
| ESP32-WROOM 38 pines      | 1        |
| Shield PCB para ESP32     | 1        |
| Shield Carga de Batería 18650 | 1    |
| Batería Lipo 3300 mAh     | 1        |
| Batería Li-ion 5000 mAh   | 1        |
| PCB Molex                 | 1        |
| Tornillos con Tuerca      | 64       |
| Estructura del robot      | 6        |


## Simulación mundo real y mundo simulado
### Lanzamiento de la Simulación en un Mundo Simulado con ROS y Gazebo
Para iniciar la simulación del mundo virtual con los robots, sigue estos pasos:
#### Paso 1: Lanzar la Simulación en Gazebo:
Abre una terminal y ejecuta el siguiente comando para lanzar Gazebo con el mundo simulado:
```
    roslaunch ros_world vera_robots_world_sim.launch
```
Esto abrirá Gazebo con el mundo virtual definido en el archivo vera_robots_world_sim.launch.

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2011-52-09.png" alt="S4">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 13.</strong> Simulación gazebo mundo simulado</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2011-56-29.png" alt="S6">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 14.</strong> Simulación gazebo mundo simulado</p>
<br>


#### Paso 2: Iniciar el Nodo de Navegación y RViz:
En una nueva terminal, ejecuta el siguiente comando para iniciar el nodo de navegación y RViz:
```
    roslaunch mrs_navigation_vera vera_robots_ros_world_sim.launch
```
Con este comando, se lanzará RViz junto con los nodos necesarios para la navegación del robot en el mundo simulado.

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2011-55-14.png" alt="S5">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 15.</strong> Rviz mundo simulado</p>
<br>

### Lanzamiento de la Simulación en Gazebo del Mundo Real con ROS
Para ejecutar la simulación que representa un entorno del mundo real, sigue estos pasos:

#### Lanzar la Simulación del Mundo Real:
Abre una terminal y ejecuta el siguiente comando para lanzar la simulación del mundo real:
```
roslaunch mrs_navigation_vera vera_robots_ros_world_sim.launch
```
Este comando iniciará la simulación en ROS que representa un entorno del mundo real, tal como está definido en el archivo vera_robots_world_real.launch.

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2011-41-50.png" alt="S1">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 16.</strong> Simulación Gazebo mundo real</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2011-40-08.png" alt="S2">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 17.</strong> Simulación Gazebo mundo real vista superior</p>
<br>

#### Iniciar el Nodo de Navegación y RViz para el Mundo Real:
En una nueva terminal, ejecuta el siguiente comando para iniciar el nodo de navegación y RViz para el entorno del mundo real:
```
    roslaunch mrs_navigation_vera vera_robots_ros_world_real.launch
```
Este comando lanzará RViz y los nodos de navegación necesarios para interactuar con el entorno simulado del mundo real.

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2011-40-39.png" alt="S3">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 18.</strong> Rviz mundo real vista superior</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2018-29-30.png" alt="S3">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 19.</strong> Espacio real de trabajo de la plataforma</p>
<br>

<img src="https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/Screenshot%20from%202024-01-13%2018-30-55.png" alt="S3">
<p style="margin-top:10px; font-size: 16px;"><strong>Figura 20.</strong> Rviz mundo real de la plataforma</p>
<br>


## Video tiktok
[![Video de ejemplo](https://github.com/VerabelGonzales/vera_multi_robot_platform/blob/main/images_robot_readme/triangulo_tiktok.png)](https://vm.tiktok.com/ZGe6F9UuE/ "Video de ejemplo")
## Ejercicios Nivel 1
## Ejercicios Nivel 2
### Coordinación Multi-Robot: Formación en Línea Diagonal con Tres Robots Omnidireccionales (Click en el video)
En el ejercicio, se observa la descripción de una trayectoria diagonal realizada por tres robots. Esta línea diagonal se mantiene constante mientras los robots avanzan a lo largo de su recorrido hasta alcanzar el final de su trayectoria. En este ejercicio se puede ver el seguimiento de lider atraves de la comunicación maestro-esclavo.

[![Ver Video](https://img.youtube.com/vi/ncjI9gKNrXw/0.jpg)](https://www.youtube.com/watch?v=ncjI9gKNrXw)
### Coordinación Multi-Robot: Formación en Línea con Tres Robots Omnidireccionales (Click en el video)
En el ejercicio, se observa la descripción de una trayectoria de linea vertical realizada por tres robots. Esta línea vertical se mantiene constante mientras los robots avanzan a lo largo de su recorrido hasta alcanzar el final de su trayectoria. En este ejercicio se puede ver el seguimiento de lider atraves de la comunicación maestro-esclavo.

[![Ver Video](https://img.youtube.com/vi/EinKM8R_A2A/0.jpg)](https://www.youtube.com/watch?v=EinKM8R_A2A)
### Coordinación Multi-Robot: Formación en Triangulo con Tres Robots Omnidireccionales (Click en el video)
En el ejercicio, se observa la formación de un triángulo realizada por tres robots. En este ejercicio se puede ver el seguimiento de lider atraves de la comunicación maestro-esclavo.

[![Ver Video](https://img.youtube.com/vi/X3KD1b3r-Cs/0.jpg)](https://www.youtube.com/watch?v=X3KD1b3r-Cs)
## Ejercicios Nivel 3
### Comunicación Multi-Robot: Tres Robots Omnidireccionales Coordinados para Mover un Objeto (Click en el video)
Este ejercicio demuestra la interacción coordinada entre tres robots para desplazar un objeto. Inicialmente, un robot intenta mover el objeto por sí solo. En caso de no lograrlo, retrocede para buscar la ayuda de un segundo robot. Si ambos, trabajando en conjunto, aún no consiguen desplazar el objeto, retroceden para incorporar a un tercer robot. Finalmente, con la fuerza combinada de los tres, logran mover exitosamente el objeto.

[![Ver Video](https://img.youtube.com/vi/sbU5sWQUMAk/0.jpg)](https://www.youtube.com/watch?v=sbU5sWQUMAk&t=1s)
### Colaboración Multi-Robot: Tres Robots Omnidireccionales Trabajando Juntos para Desplazar un Objeto (Click en el video)
Este ejercicio ilustra la colaboración efectiva de tres robots para desplazar un objeto en línea recta, asegurando que no se produzcan desviaciones. Inicialmente, se observa el desplazamiento del objeto a lo largo del eje 'x'. A continuación, el movimiento se dirige por el eje 'y', seguido de otro desplazamiento a lo largo del eje 'x'. Finalmente, el objeto se mueve una vez más en el eje 'y'. Este patrón demuestra la precisión y la coordinación en el trabajo conjunto de los robots.

[![Ver Video](https://img.youtube.com/vi/lTV28RFhZQM/0.jpg)](https://www.youtube.com/watch?v=lTV28RFhZQM)
