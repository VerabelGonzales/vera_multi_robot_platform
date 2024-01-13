#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np

class IRtoLidarNode:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('ir_to_lidar_node')

        # Suscriptores para los sensores IR
        self.ir_subscriber_robot_1 = rospy.Subscriber("robot_1/ir_distances", Float32MultiArray, self.ir_callback_robot_1)
        self.ir_subscriber_robot_2 = rospy.Subscriber("robot_2/ir_distances", Float32MultiArray, self.ir_callback_robot_2)
        self.ir_subscriber_robot_3 = rospy.Subscriber("robot_3/ir_distances", Float32MultiArray, self.ir_callback_robot_3)

        # Publicadores para el Lidar 2D, uno para cada robot
        self.lidar_publisher_robot_1 = rospy.Publisher("robot_1/scan", LaserScan, queue_size=10)
        self.lidar_publisher_robot_2 = rospy.Publisher("robot_2/scan", LaserScan, queue_size=10)
        self.lidar_publisher_robot_3 = rospy.Publisher("robot_3/scan", LaserScan, queue_size=10)

    # Callbacks para cada robot
    def ir_callback_robot_1(self, data):
        self.publish_lidar_data_1(data, self.lidar_publisher_robot_1)

    def ir_callback_robot_2(self, data):
        self.publish_lidar_data_2(data, self.lidar_publisher_robot_2)

    def ir_callback_robot_3(self, data):
        self.publish_lidar_data_3(data, self.lidar_publisher_robot_3)


    def publish_lidar_data_1(self, data, publisher):
        # Convertir datos IR a formato LaserScan
        laser_scan = LaserScan()
        laser_scan.header.stamp = rospy.Time.now()
        laser_scan.header.frame_id = "robot_1/base_footprint"  # Ajustar según el frame_id correspondiente

        # Configuración del LaserScan
        num_readings = 360  # Número de lecturas en una revolución completa
        laser_scan.angle_min = -np.pi
        laser_scan.angle_max = np.pi
        laser_scan.angle_increment = 2 * np.pi / num_readings
        laser_scan.time_increment = 0  # Ajustar si es conocido
        laser_scan.range_min = 0.10  # Ajustar según el rango mínimo de los sensores IR
        laser_scan.range_max = 0.80 # Ajustar según el rango máximo de los sensores IR

        # Inicializar el arreglo de rangos con todos los valores en cero
        laser_scan.ranges = [0.0] * num_readings

        # Asignar lecturas de los sensores IR a las direcciones correspondientes
        # La asignación exacta de los ángulos dependerá de la orientación de tus sensores
        # Aquí hay un ejemplo básico:
        laser_scan.ranges[90] = (data.data[0]/100.0)  # Sensor IR 1 hacia adelante
        laser_scan.ranges[0] = (data.data[1]/100.0)   # Sensor IR 2 hacia la derecha
        laser_scan.ranges[180] = (data.data[3]/100.0) # Sensor IR 4 hacia la izquierda
        laser_scan.ranges[270] = (data.data[2]/100.0) # Sensor IR 3 hacia atrás

        # Publicar datos en el tópico del Lidar 2D
        publisher.publish(laser_scan)


    def publish_lidar_data_2(self, data, publisher):
        # Convertir datos IR a formato LaserScan
        laser_scan = LaserScan()
        laser_scan.header.stamp = rospy.Time.now()
        laser_scan.header.frame_id = "robot_2/base_footprint"  # Ajustar según el frame_id correspondiente

        # Configuración del LaserScan
        num_readings = 360  # Número de lecturas en una revolución completa
        laser_scan.angle_min = -np.pi
        laser_scan.angle_max = np.pi
        laser_scan.angle_increment = 2 * np.pi / num_readings
        laser_scan.time_increment = 0  # Ajustar si es conocido
        laser_scan.range_min = 0.10  # Ajustar según el rango mínimo de los sensores IR
        laser_scan.range_max = 0.80 # Ajustar según el rango máximo de los sensores IR

        # Inicializar el arreglo de rangos con todos los valores en cero
        laser_scan.ranges = [0.0] * num_readings

        # Asignar lecturas de los sensores IR a las direcciones correspondientes
        # La asignación exacta de los ángulos dependerá de la orientación de tus sensores
        # Aquí hay un ejemplo básico:
        laser_scan.ranges[90] = (data.data[0]/100)  # Sensor IR 1 hacia adelante
        laser_scan.ranges[0] = (data.data[1]/100.0)   # Sensor IR 2 hacia la derecha
        laser_scan.ranges[180] = (data.data[3]/100.0)# Sensor IR 4 hacia la izquierda
        laser_scan.ranges[270] = (data.data[2]/100.0) # Sensor IR 3 hacia atrás

        # Publicar datos en el tópico del Lidar 2D
        publisher.publish(laser_scan)

    def publish_lidar_data_3(self, data, publisher):
        # Convertir datos IR a formato LaserScan
        laser_scan = LaserScan()
        laser_scan.header.stamp = rospy.Time.now()
        laser_scan.header.frame_id = "robot_3/base_footprint"  # Ajustar según el frame_id correspondiente

        # Configuración del LaserScan
        num_readings = 360  # Número de lecturas en una revolución completa
        laser_scan.angle_min = -np.pi
        laser_scan.angle_max = np.pi
        laser_scan.angle_increment = 2 * np.pi / num_readings
        laser_scan.time_increment = 0  # Ajustar si es conocido
        laser_scan.range_min = 0.10  # Ajustar según el rango mínimo de los sensores IR
        laser_scan.range_max = 0.80 # Ajustar según el rango máximo de los sensores IR

        # Inicializar el arreglo de rangos con todos los valores en cero
        laser_scan.ranges = [0.0] * num_readings

        # Asignar lecturas de los sensores IR a las direcciones correspondientes
        # La asignación exacta de los ángulos dependerá de la orientación de tus sensores
        # Aquí hay un ejemplo básico:
        laser_scan.ranges[90] = (data.data[0]/100.0)  # Sensor IR 1 hacia adelante
        laser_scan.ranges[0] = (data.data[1]/100.0)   # Sensor IR 2 hacia la derecha
        laser_scan.ranges[180] = (data.data[3]/100.0) # Sensor IR 4 hacia la izquierda
        laser_scan.ranges[270] = (data.data[2]/100.0) # Sensor IR 3 hacia atrás

        # Publicar datos en el tópico del Lidar 2D
        publisher.publish(laser_scan)

if __name__ == '__main__':
    try:
        node = IRtoLidarNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
