#!/usr/bin/env python3

import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from math import pow, atan2, sqrt
from math import radians, degrees
import sys, select, termios, tty
import tf
import math
import threading


class VeraRobot(threading.Thread):

    def __init__(self, rate):

        super(VeraRobot, self).__init__()

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)

        self.odom_data = None

        self.pose = Pose2D()
        self.rate = rospy.Rate(10)

        self.condition = threading.Condition()
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True

        #Establecer timeout a None si rate es 0 (hace que new_message espere eternamente a que se publiquen nuevos datos)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.publish_thread.start()
        self.lock = threading.Lock()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.velocity_publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
        
    def wait_for_odom_data(self):
        while self.odom_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            print("hola")
        return
     
    def update_pose(self, data):
        """Cuando el suscriptor recibe nuevo mensaje se actualiza la posicion."""
        self.odom_data = data
        #Obtenemos Poscicion 
        position = data.pose.pose.position

        #Obtenemos Orientacion       
        orientation = data.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        
        self.pose.x = round(position.x, 4)
        self.pose.y = round(position.y, 4)
        self.pose.theta = yaw

    def euclidean_distance(self, goal_pose):
        """Distancia euclidiana entre la posicion actual y la meta."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def steering_angle(self, goal_pose):
        """Direccion del angulo."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def linear_vel(self, goal_pose, constant=0.1): 
    
        offset = 0.5 
        vel = constant * self.euclidean_distance(goal_pose) + offset
        if vel >= 0.3:
           vel = 0.3
        return vel

    def angular_vel(self, goal_pose, constant=1.1): 
         goal_angle = self.steering_angle(goal_pose)
         #current_angle = self.pose.theta 
         current_angle = math.fmod(self.pose.theta, 2*math.pi)
         return constant * (math.fmod((goal_angle - current_angle + math.pi),(2 * math.pi)) - math.pi)   
    
    def publish_loop(self):
        try:
            rate = rospy.Rate(20)  # 10 Hz
            while not rospy.is_shutdown():
                with self.condition:
                    self.condition.wait()  # Esperar hasta que se actualice el comando
                    self.vel_publisher.publish(self.command)
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("ROS interrupt received, shutting down publish loop")
        except Exception as e:
            rospy.logerr(f"Error in publish loop: {e}")


    # Función para orientar el robot a un ángulo específico
    def orient_to_angle(self, desired_angle):
        """Gira el robot hasta alcanzar el ángulo deseado."""
        angle_tolerance = 0.05  # Tolerancia de ángulo en radianes
        vel_msg = Twist()

        while not rospy.is_shutdown():
            current_angle = self.pose.theta
            angle_diff = desired_angle - current_angle

            # Verifica si el ángulo está dentro de la tolerancia
            if abs(angle_diff) < angle_tolerance:
                break

            vel_msg.angular.z = 1.5 * angle_diff  # Ajusta este factor según sea necesario
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Detiene la rotación
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    # Modificación de la función move2goal para mantener constante la orientación horizontal

    def move2goal(self):
        """Mover hacia el objetivo con movimiento en los ejes X e Y, manteniendo constante la orientación."""
        goal_pose = Pose2D()

        # Obtiene las coordenadas del usuario
        point_x, point_y = input("Ingrese las coordenadas separadas por coma (x,y): ").split(",")
        goal_pose.x = float(point_x)
        goal_pose.y = float(point_y)

        # Tolerancia para la distancia en cada eje
        tolerance = 0.15
        # Tolerancia de ángulo en radianes
        angle_tolerance = 0.05

        vel_msg = Twist()

        while not rospy.is_shutdown():
            # Verificar y ajustar la orientación
            current_angle = self.pose.theta
            if abs(current_angle) > angle_tolerance:
                vel_msg.angular.z = 1.5 * -current_angle  # Ajusta este factor según sea necesario
            else:
                vel_msg.angular.z = 0

            # Mover en el eje X
            if abs(goal_pose.x - self.pose.x) >= tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose) * (1 if goal_pose.x > self.pose.x else -1)
            else:
                vel_msg.linear.x = 0

            # Mover en el eje Y
            if abs(goal_pose.y - self.pose.y) >= tolerance:
                vel_msg.linear.y = self.linear_vel(goal_pose) * (1 if goal_pose.y > self.pose.y else -1)
            else:
                vel_msg.linear.y = 0

            # Publicar velocidades
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            # Verificar si el objetivo ha sido alcanzado
            if abs(goal_pose.x - self.pose.x) < tolerance and abs(goal_pose.y - self.pose.y) < tolerance:
                break

        # Detener el robot
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        answer = input("¿Quieres ingresar otro punto? y/n:  ")
        if answer == "y":
            self.move2goal()
        else:
            rospy.signal_shutdown("Programa finalizado")
            sys.exit()

    # Nota: Este código es un fragmento y debe ser integrado en la clase VeraRobot.
    # Puede ser necesario ajustar las constantes y la lógica según las especificaciones del robot real.


if __name__ == '__main__':
    
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('goal_to_postion')
    repeat = rospy.get_param("~repeat_rate", 0.0)
    goal_position = VeraRobot(repeat)

    try:
        goal_position.wait_for_subscribers()
        goal_position.wait_for_odom_data()

        rospy.loginfo("Inicio el node")
              
        goal_position.move2goal()

    except KeyboardInterrupt:
        print("Shutting down")

