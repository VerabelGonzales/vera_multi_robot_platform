#!/usr/bin/env python
import rospy
import threading
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from actionlib_msgs.msg import GoalID
from math import pow, atan2, sqrt, degrees, pi, cos, sin
import tf

class VeraRobot:

    DISTANCE_TOLERANCE = 0.12  # Tolerancia en el eje X y Y.
    ANGULAR_TOLERANCE = 0.05 # Tolerancia angular.

    VEL_LINEAL_MAX = 0.22 #Velocidad maxima en el eje X y Y.
    VEL_ANGULAR_MAX = 0.8 #Velocidad angular maxima.

    K = 0.15 # Constante proporcional.
    L = 0.1 # Distancia entre los ejes de las ruedas al centro del robot. 

    def __init__(self):
        rospy.init_node('vera_robots_controller', anonymous=True)

        # Variables para almacenar los datos de odoemtria de todos los robots
        self.odom_data_robot_1 = None
        self.odom_data_robot_2 = None
        self.odom_data_robot_3 = None
        # Variables para almacenar la posicion y orietacion el robot
        self.odom_data_object = None
        self.position_object = None
        self.orientation_object = None

        # Inicialización de variables para almacenar la retroalimentación de los resultados de los robots.
        # Se utiliza la clase MoveBaseActionResult para almacenar el estado del resultado de las acciones de movimiento.
        self.result_data_robot_1 = MoveBaseActionResult()
        self.result_data_robot_2 = MoveBaseActionResult()
        self.result_data_robot_3 = MoveBaseActionResult()

        # Variables para almacenar la posicion y orientacion de los robots y el objeto.
        # Pose2D es una estructura de datos simple que contiene tres campos: x, y y theta. 
        self.pose_robot_1 = Pose2D()
        self.pose_robot_2 = Pose2D()
        self.pose_robot_3 = Pose2D()
        self.pose_object = Pose2D()

        # Instancias para comandos de movimiento de los robots móviles.
        # Utilizamos la clase Twist, donde `linear` representa la velocidad lineal y `angular` la velocidad angular.
        self.command_robot_1 = Twist()
        self.command_robot_2 = Twist()
        self.command_robot_3 = Twist()

        # Configuración de un hilo de ejecución para la publicación de velocidades de todos los robots. .
        # Se usa un objeto de condición para sincronizar la publicación de velocidades.
        self.condition_robot_1 = threading.Condition()  # Objeto de condición para controlar el acceso a recursos compartidos.
        self.publish_thread_robot_1 = threading.Thread(target=self.publish_loop_robot_1)  # Creación del hilo para publicar velocidades.
        self.publish_thread_robot_1.daemon = True  # Configuración del hilo como demonio para que se cierre automáticamente cuando el programa termina.
        self.publish_thread_robot_1.start()  # Inicio del hilo de publicación de velocidades.

         #Condiciones para que la publicacion de velocidades del robot 2
        self.condition_robot_2 = threading.Condition()
        self.publish_thread_robot_2 = threading.Thread(target=self.publish_loop_robot_2)
        self.publish_thread_robot_2.daemon = True
        self.publish_thread_robot_2.start()

        #Condiciones para que la publicacion de velocidades del robot 3
        self.condition_robot_3 = threading.Condition()
        self.publish_thread_robot_3 = threading.Thread(target=self.publish_loop_robot_3)
        self.publish_thread_robot_3.daemon = True
        self.publish_thread_robot_3.start()

        # Suscriptores para para obtener los datos de odometria de nuestros robots y el objeto
        self.odom_subscriber_robot_1 = rospy.Subscriber("robot_1/odom", Odometry, self.update_pose_robot_1)
        self.odom_subscriber_robot_2 = rospy.Subscriber("robot_2/odom", Odometry, self.update_pose_robot_2)
        self.odom_subscriber_robot_3 = rospy.Subscriber("robot_3/odom", Odometry, self.update_pose_robot_3)
        self.odom_subscriber_objeto = rospy.Subscriber("objeto/odom", Odometry, self.update_pose_objeto)
        #Publicadores para la asignacion de las velocidades para los robots
        self.vel_robot_1_publisher = rospy.Publisher("robot_1/cmd_vel", Twist, queue_size=10)
        self.vel_robot_2_publisher = rospy.Publisher("robot_2/cmd_vel", Twist, queue_size=10)
        self.vel_robot_3_publisher = rospy.Publisher("robot_3/cmd_vel", Twist, queue_size=10)
        # Publicadores de posicion para navegacion con los robots. 
        self.goal_robot_1_publisher = rospy.Publisher("robot_1/move_base_simple/goal", PoseStamped, queue_size=10)
        self.goal_robot_2_publisher = rospy.Publisher("robot_2/move_base_simple/goal", PoseStamped, queue_size=10)
        self.goal_robot_3_publisher = rospy.Publisher("robot_3/move_base_simple/goal", PoseStamped, queue_size=10)
        # Fedback resuls
        self.result_subscriber_robot_1 = rospy.Subscriber("robot_1/move_base/result", MoveBaseActionResult, self.result_callback_robot_1)
        self.result_subscriber_robot_2 = rospy.Subscriber("robot_2/move_base/result", MoveBaseActionResult, self.result_callback_robot_2)
        self.result_subscriber_robot_3 = rospy.Subscriber("robot_3/move_base/result", MoveBaseActionResult, self.result_callback_robot_3)
        # Publicador en el topico para cancelar la navegacion.
        self.cancel_goal_robot_1 = rospy.Publisher("robot_1/move_base/cancel", GoalID, queue_size=10)
        self.cancel_goal_robot_2 = rospy.Publisher("robot_1/move_base/cancel", GoalID, queue_size=10)
        self.cancel_goal_robot_3 = rospy.Publisher("robot_1/move_base/cancel", GoalID, queue_size=10)
       
    def wait_for_subscribers_cmd_vel(self):
        try:
            while not rospy.is_shutdown() and self.vel_robot_1_publisher.get_num_connections() == 0 and self.vel_robot_2_publisher.get_num_connections() == 0 and self.vel_robot_3_publisher.get_num_connections() == 0:
                rospy.sleep(0.1)
            if rospy.is_shutdown():
                raise rospy.exceptions.ROSInterruptException("Shutdown requested before subscribers connected")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(f"Error while waiting for subscribers: {e}")
    
    def wait_for_subscribers_move_base(self):
        try:
            while not rospy.is_shutdown() and self.goal_robot_1_publisher.get_num_connections() == 0 and self.goal_robot_2_publisher.get_num_connections() == 0 and self.goal_robot_3_publisher.get_num_connections() == 0:
                rospy.sleep(0.1)
            if rospy.is_shutdown():
                raise rospy.exceptions.ROSInterruptException("Shutdown requested before subscribers connected")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(f"Error while waiting for subscribers: {e}")
    

    def wait_for_odom_data_robot_1(self):
        while self.odom_data_robot_1 is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        # Agregar una impresión para confirmar que se recibieron los datos
        rospy.loginfo(f"Datos de odometría recibidos para robot 1")
        return
    
    def wait_for_odom_data_robot_2(self):
        while self.odom_data_robot_2 is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        # Agregar una impresión para confirmar que se recibieron los datos
        rospy.loginfo(f"Datos de odometría recibidos para robot 2")
        return
    
    def wait_for_odom_data_robot_3(self):
        while self.odom_data_robot_3 is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        # Agregar una impresión para confirmar que se recibieron los datos
        rospy.loginfo(f"Datos de odometría recibidos para robot 3")
        return
    
    def wait_for_odom_data_objeto(self):
        while self.odom_data_object is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        # Agregar una impresión para confirmar que se recibieron los datos
        rospy.loginfo(f"Datos de odometría recibidos para objeto")
        return
    

    def update_command_robot_1(self, linear_x, linear_y, angular_z):
        with self.condition_robot_1:
            self.command_robot_1.linear.x = linear_x
            self.command_robot_1.linear.y = linear_y
            self.command_robot_1.angular.z = angular_z
            self.condition_robot_1.notify()
    
    def update_command_robot_2(self, linear_x, linear_y, angular_z):
        with self.condition_robot_2:
            self.command_robot_2.linear.x = linear_x
            self.command_robot_2.linear.y = linear_y
            self.command_robot_2.angular.z = angular_z
            self.condition_robot_2.notify()
    
    def update_command_robot_3(self, linear_x, linear_y, angular_z):
        with self.condition_robot_3:
            self.command_robot_3.linear.x = linear_x
            self.command_robot_3.linear.y = linear_y
            self.command_robot_3.angular.z = angular_z
            self.condition_robot_3.notify()


    def result_callback_robot_1(self, data):
        self.result_data_robot_1 = data
        if self.result_data_robot_1.status.status == 3:  # El código 3 significa éxito
            rospy.loginfo("Robot 1 ha llegado a su objetivo.")
            self.stop_robot_1()
  
    def result_callback_robot_2(self, data):
        self.result_data_robot_2 = data
        if self.result_data_robot_2.status.status == 3:
            rospy.loginfo("Robot 2 ha llegado a su objetivo.")
            self.stop_robot_2()

    def result_callback_robot_3(self, data):
        self.result_data_robot_3 = data
        if self.result_data_robot_3.status.status == 3:
            rospy.loginfo("Robot 3 ha llegado a su objetivo.")
            self.stop_robot_3()


    def stop_robot_1(self):
        for _ in range(10):
            self.update_command_robot_1(0, 0, 0)
    
    def stop_robot_2(self):
        for _ in range(10):
            self.update_command_robot_2(0, 0, 0)
    
    def stop_robot_3(self):
        for _ in range(10):
            self.update_command_robot_3(0, 0, 0)


    def publish_loop_robot_1(self):
        try:
            rate = rospy.Rate(20)  # 10 Hz
            while not rospy.is_shutdown():
                with self.condition_robot_1:
                    self.condition_robot_1.wait()  # Esperar hasta que se actualice el comando
                    self.vel_robot_1_publisher.publish(self.command_robot_1)
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("ROS interrupt received, shutting down publish loop")
        except Exception as e:
            rospy.logerr(f"Error in publish loop: {e}")
    
    def publish_loop_robot_2(self):
        try:
            rate = rospy.Rate(20)  # 10 Hz
            while not rospy.is_shutdown():
                with self.condition_robot_2:
                    self.condition_robot_2.wait()  # Esperar hasta que se actualice el comando
                    self.vel_robot_2_publisher.publish(self.command_robot_2)
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("ROS interrupt received, shutting down publish loop")
        except Exception as e:
            rospy.logerr(f"Error in publish loop: {e}")
    
    def publish_loop_robot_3(self):
        try:
            rate = rospy.Rate(20)  # 10 Hz
            while not rospy.is_shutdown():
                with self.condition_robot_3:
                    self.condition_robot_3.wait()  # Esperar hasta que se actualice el comando
                    self.vel_robot_3_publisher.publish(self.command_robot_3)
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("ROS interrupt received, shutting down publish loop")
        except Exception as e:
            rospy.logerr(f"Error in publish loop: {e}")


    def update_pose_robot_1(self, data):
        # Cuando el suscriptor recibe nuevo mensaje se actualiza la posicion.
        self.odom_data_robot_1 = data
        #Obtenemos Poscicion 
        position = data.pose.pose.position
        #Obtenemos Orientacion       
        orientation = data.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        #Posicion y orietacion del robot 1
        self.pose_robot_1.x = position.x
        self.pose_robot_1.y = position.y
        self.pose_robot_1.theta = yaw
    
    def update_pose_robot_2(self, data):
        # Cuando el suscriptor recibe nuevo mensaje se actualiza la posicion.
        self.odom_data_robot_2 = data
        #Obtenemos Poscicion 
        position = data.pose.pose.position
        #Obtenemos Orientacion       
        orientation = data.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        #Posicion y orietacion del robot 2
        self.pose_robot_2.x = position.x
        self.pose_robot_2.y = position.y
        self.pose_robot_2.theta = yaw
    
    def update_pose_robot_3(self, data):
        # Cuando el suscriptor recibe nuevo mensaje se actualiza la posicion.
        self.odom_data_robot_3 = data
        #Obtenemos Poscicion 
        position = data.pose.pose.position
        #Obtenemos Orientacion       
        orientation = data.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        #Posicion y orietacion del robot 2
        self.pose_robot_3.x = position.x
        self.pose_robot_3.y = position.y
        self.pose_robot_3.theta = yaw
    
    def update_pose_objeto(self, data):

        self.position_object = data.pose.pose.position
        self.orientation_object = data.pose.pose.orientation

        self.odom_data_object = data
        #Obtenemos Poscicion 
        position = data.pose.pose.position
        #Obtenemos Orientacion       
        orientation = data.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        self.pose_object.x = position.x
        self.pose_object.y = position.y
        self.pose_object.theta = yaw


    def euclidean_distance(self, robot_id):
        # Distancia euclidiana entre la posicion actual y la meta.
        if robot_id == 1:
            return sqrt(pow((self.goal_1.pose.position.x - self.pose_robot_1.x), 2) +
                    pow((self.goal_1.pose.position.y - self.pose_robot_1.y), 2))
        
        if robot_id == 2:
            return sqrt(pow((self.goal_2.pose.position.x - self.pose_robot_2.x), 2) +
                    pow((self.goal_2.pose.position.y - self.pose_robot_2.y), 2))
        
        if robot_id == 3:
            return sqrt(pow((self.goal_3.pose.position.x - self.pose_robot_3.x), 2) +
                    pow((self.goal_3.pose.position.y - self.pose_robot_3.y), 2))
        
    def euclidean_distance_2(self, robot_id, goal):
        # Distancia euclidiana entre la posicion actual y la meta.
        if robot_id == 1:
            return sqrt(pow((goal.x - self.pose_robot_1.x), 2) +
                    pow((goal.y - self.pose_robot_1.y), 2))
        
        if robot_id == 2:
            return sqrt(pow((goal.x - self.pose_robot_2.x), 2) +
                    pow((goal.y - self.pose_robot_2.y), 2))
        
        if robot_id == 3:
            return sqrt(pow((goal.x - self.pose_robot_3.x), 2) +
                    pow((goal.y - self.pose_robot_3.y), 2))
        
    def linear_vel(self, goal_pose, constant, robot_id): 
    
        offset = 0.5
        if robot_id == 1:
            vel = constant * self.euclidean_distance_2(1, goal_pose) + offset
            if vel >= 0.08:
                vel = 0.08
            return vel
        
        if robot_id == 2:
            vel = constant * self.euclidean_distance_2(2, goal_pose) + offset
            if vel >= 0.08:
                vel = 0.08
            return vel
        
        if robot_id == 3:
            vel = constant * self.euclidean_distance_2(3, goal_pose) + offset
            if vel >= 0.08:
                vel = 0.08
            return vel


    def orient_to_angle_robot_1_3(self, desired_angle):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():

            angle_diff_1 = desired_angle - self.pose_robot_1.theta
            angle_diff_3 = desired_angle - self.pose_robot_3.theta
            
            if abs(angle_diff_1) < VeraRobot.ANGULAR_TOLERANCE and (angle_diff_3) < VeraRobot.ANGULAR_TOLERANCE:
                print("Ingreso a angulo 1")
                print("Ingreso a angulo 3")
                break

            self.update_command_robot_1(0.0, 0.0, 0.50 * angle_diff_1)
            self.update_command_robot_1(0.0, 0.0, 0.50 * angle_diff_3)

            rate.sleep()

        # Detener todos los robots por seguridad.
        self.stop_robot_1()
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_1()
        self.stop_robot_3()
    
    def orient_to_angle_robot_2(self, desired_angle):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            
            current_angle = self.pose_robot_2.theta
            angle_diff = desired_angle - current_angle
            
            if abs(angle_diff) < VeraRobot.ANGULAR_TOLERANCE:
                print("Ingreso a angulo 2")
                break

            self.update_command_robot_2(0.0, 0.0, 0.50 * angle_diff)

            rate.sleep()

        # Detener todos los robots por seguridad.
        self.stop_robot_2()
        rospy.sleep(1)
        self.stop_robot_2()

    def orient_to_angle_robot_3(self, desired_angle):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():

            current_angle = self.pose_robot_3.theta
            angle_diff = desired_angle - current_angle
            
            if abs(angle_diff) < VeraRobot.ANGULAR_TOLERANCE:
                print("Ingreso a angulo 3")
                break

            self.update_command_robot_3(0.0, 0.0, 0.50 * angle_diff)

            rate.sleep()

        # Detener todos los robots por seguridad.
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_3()


    def send_goals_to_object(self, robot_id):
        if self.position_object is None or self.orientation_object is None:
            rospy.logwarn("Posición del objeto no recibida todavía.")
            return

        offset = 0.30

        self.goal_1 = PoseStamped()
        self.goal_1.header.stamp = rospy.Time.now()
        self.goal_1.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame

        self.goal_2 = PoseStamped()
        self.goal_2.header.stamp = rospy.Time.now()
        self.goal_2.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame
        
        self.goal_3 = PoseStamped()
        self.goal_3.header.stamp = rospy.Time.now()
        self.goal_3.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame
        
        # Calcular y enviar las metas para cada robot
        if robot_id == 1:
            self.goal_1.pose.position.x = self.position_object.x 
            self.goal_1.pose.position.y = self.position_object.y + offset
            self.goal_1.pose.position.z = self.position_object.z
            self.goal_1.pose.orientation = self.orientation_object
            self.goal_robot_1_publisher.publish(self.goal_1)

        elif robot_id == 2:
            self.goal_2.pose.position.x = self.position_object.x - 0.35
            self.goal_2.pose.position.y = self.position_object.y
            self.goal_2.pose.position.z = self.position_object.z
            self.goal_2.pose.orientation = self.orientation_object
            self.goal_robot_2_publisher.publish(self.goal_2)

        elif robot_id == 3:
            self.goal_3.pose.position.x = self.position_object.x 
            self.goal_3.pose.position.y = self.position_object.y - offset
            self.goal_3.pose.position.z = self.position_object.z
            self.goal_3.pose.orientation = self.orientation_object
            self.goal_robot_3_publisher.publish(self.goal_3)
    
    def send_goals(self, robot_id, goal_x, goal_y):
        if self.position_object is None or self.orientation_object is None:
            rospy.logwarn("Posición del objeto no recibida todavía.")
            return

        offset = 0.30

        self.goal_1 = PoseStamped()
        self.goal_1.header.stamp = rospy.Time.now()
        self.goal_1.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame

        self.goal_2 = PoseStamped()
        self.goal_2.header.stamp = rospy.Time.now()
        self.goal_2.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame
        
        self.goal_3 = PoseStamped()
        self.goal_3.header.stamp = rospy.Time.now()
        self.goal_3.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame
        
        # Calcular y enviar las metas para cada robot
        if robot_id == 1:
            self.goal_1.pose.position.x = self.pose_robot_1.x + (goal_x)
            self.goal_1.pose.position.y = self.pose_robot_1.y + (goal_y)
            self.goal_1.pose.position.z = 0.0
            self.goal_1.pose.orientation.x = 0.0
            self.goal_1.pose.orientation.y = 0.0
            self.goal_1.pose.orientation.z = 0.0
            self.goal_1.pose.orientation.w = 1.0
            self.goal_robot_1_publisher.publish(self.goal_1)

        elif robot_id == 2:
            self.goal_2.pose.position.x = self.pose_robot_2.x + (goal_x)
            self.goal_2.pose.position.y = self.pose_robot_2.y + (goal_y)
            self.goal_2.pose.position.z = 0.0
            self.goal_2.pose.orientation.x = 0.0
            self.goal_2.pose.orientation.y = 0.0
            self.goal_2.pose.orientation.z = 0.0
            self.goal_2.pose.orientation.w = 1.0
            self.goal_robot_2_publisher.publish(self.goal_2)

        elif robot_id == 3:
            self.goal_3.pose.position.x = self.pose_robot_3.x + (goal_x)
            self.goal_3.pose.position.y = self.pose_robot_3.y + (goal_y)
            self.goal_3.pose.position.z = 0.0
            self.goal_3.pose.orientation.x = 0.0
            self.goal_3.pose.orientation.y = 0.0
            self.goal_3.pose.orientation.z = 0.0
            self.goal_3.pose.orientation.w = 1.0
            self.goal_robot_3_publisher.publish(self.goal_3)


    def velocity_controller_robot_1(self):
        # Inicializar variables
        error_x = 0  # Error en la dirección X
        error_y = 0  # Error en la dirección Y
        linear_velocity = 0  # Velocidad lineal del robot
        angular_velocity = 0  # Velocidad angular del robot

        # Utilizando las posiciones deseadas proporcionadas como argumentos de la función
        distance_ahead = 0.38  # Distancia deseada hacia adelante (50 cm)
        current_x = self.pose_robot_1.x  # Posición actual en X
        current_y = self.pose_robot_1.y  # Posición actual en Y
        desired_x = current_x + distance_ahead  # Posición deseada en X
        desired_y = current_y  # Posición deseada en Y

        point_x = self.pose_robot_1.x + 0.1 * cos(self.pose_robot_1.theta)  # Coordenadas del punto externo en X
        point_y = self.pose_robot_1.y + 0.1 * sin(self.pose_robot_1.theta)  # Coordenadas del punto externo en Y
        error_x = point_x - desired_x  # Calcular error en X
        error_y = point_y - desired_y  # Calcular error en Y

        # Controlador cinemático
        control_x = -0.15 * error_x  # Controlador para la dirección X
        control_y = -0.15 * error_y  # Controlador para la dirección Y

        # Calcular las velocidades según el modelo cinemático para un robot móvil tipo diferencial
        linear_velocity = control_x * cos(self.pose_robot_1.theta) + control_y * sin(self.pose_robot_1.theta)
        angular_velocity = (-control_x * sin(self.pose_robot_1.theta) / 0.1) + (control_y * cos(self.pose_robot_1.theta) / 0.1)

        # Saturación de velocidades
        if abs(linear_velocity) > VeraRobot.VEL_LINEAL_MAX:
            linear_velocity = VeraRobot.VEL_LINEAL_MAX * abs(linear_velocity) / linear_velocity

        if abs(angular_velocity) > VeraRobot.VEL_ANGULAR_MAX:
            angular_velocity = VeraRobot.VEL_ANGULAR_MAX * abs(angular_velocity) / angular_velocity

        return linear_velocity, angular_velocity

    def velocity_controller_robot_2(self):
        # Inicializar variables
        error_x = 0  # Error en la dirección X
        error_y = 0  # Error en la dirección Y
        linear_velocity = 0  # Velocidad lineal del robot
        angular_velocity = 0  # Velocidad angular del robot

        distance_ahead = 0.60 # Distancia deseada hacia adelante (50 cm)
        # Usar las posiciones deseadas proporcionadas como parámetros
        current_x = self.pose_robot_2.x  # Posición actual en X
        current_y = self.pose_robot_2.y  # Posición actual en Y
        desired_x = current_x + distance_ahead  # Posición deseada en X
        desired_y = current_y   # Posición deseada en Y

        point_x = self.pose_robot_2.x + VeraRobot.L * cos(self.pose_robot_2.theta)  # Coordenadas del punto externo en X
        point_y = self.pose_robot_2.y + VeraRobot.L * sin(self.pose_robot_2.theta)  # Coordenadas del punto externo en Y
        error_x = point_x - desired_x  # Calcular error en X
        error_y = point_y - desired_y  # Calcular error en Y

        # Controlador cinemático
        control_x = -VeraRobot.K * error_x  # Controlador para la dirección X
        control_y = -VeraRobot.K * error_y  # Controlador para la dirección Y

        # Calcular las velocidades según el modelo cinemático para un robot móvil tipo diferencial
        linear_velocity = control_x * cos(self.pose_robot_2.theta) + control_y * sin(self.pose_robot_2.theta)
        angular_velocity = (-control_x * sin(self.pose_robot_2.theta) / VeraRobot.L) + (control_y * cos(self.pose_robot_2.theta) / VeraRobot.L)

        # Saturación de velocidades
        if abs(linear_velocity) > VeraRobot.VEL_LINEAL_MAX - 0.05:
            linear_velocity = VeraRobot.VEL_LINEAL_MAX * abs(linear_velocity) / linear_velocity

        if abs(angular_velocity) > VeraRobot.VEL_ANGULAR_MAX - 0.4:
            angular_velocity = VeraRobot.VEL_ANGULAR_MAX * abs(angular_velocity) / angular_velocity
        
        return linear_velocity, angular_velocity

    def velocity_controller_robot_3(self):
        # Inicializar variables
        error_x = 0  # Error en la dirección X
        error_y = 0  # Error en la dirección Y
        linear_velocity = 0  # Velocidad lineal del robot
        angular_velocity = 0  # Velocidad angular del robot
        desired_x = 0  # Posición deseada en X
        desired_y = 0  # Posición deseada en Y

        distance_ahead = 0.5  # Distancia deseada hacia adelante (50 cm)
        current_x = self.pose_robot_3.x  # Posición actual en X
        current_y = self.pose_robot_3.y  # Posición actual en Y
        desired_x = current_x + distance_ahead  # Posición deseada en X (50 cm adelante)
        desired_y = current_y  # Posición deseada en Y (misma posición Y)

        point_x = self.pose_robot_3.x + VeraRobot.L * cos(self.pose_robot_3.theta)  # Coordenadas del punto externo en X
        point_y = self.pose_robot_3.y + VeraRobot.L * sin(self.pose_robot_3.theta)  # Coordenadas del punto externo en Y
        error_x = point_x - desired_x  # Calcular error en X
        error_y = point_y - desired_y  # Calcular error en Y

        # Controlador cinemático
        control_x = -VeraRobot.K * error_x  # Controlador para la dirección X
        control_y = -VeraRobot.K * error_y  # Controlador para la dirección Y

        # Calcular las velocidades según el modelo cinemático para un robot móvil tipo diferencial
        linear_velocity = control_x * cos(self.pose_robot_3.theta) + control_y * sin(self.pose_robot_3.theta)
        angular_velocity = (-control_x * sin(self.pose_robot_3.theta) / VeraRobot.L) + (control_y * cos(self.pose_robot_3.theta) / VeraRobot.L)

        # Saturación de velocidades
        if abs(linear_velocity) > VeraRobot.VEL_LINEAL_MAX:
            linear_velocity = VeraRobot.VEL_LINEAL_MAX * abs(linear_velocity) / linear_velocity

        if abs(angular_velocity) > VeraRobot.VEL_ANGULAR_MAX:
            angular_velocity = VeraRobot.VEL_ANGULAR_MAX * abs(angular_velocity) / angular_velocity
        
        return linear_velocity, angular_velocity


    def move_object(self):
        # -------------------------------------------------------------- PRIMER CICLO -------------------------------------------------
        # Sostener objeto
        self.update_command_robot_1(0.0, -0.25, 0.0)
        self.update_command_robot_2(0.20, 0.0, 0.0)
        self.update_command_robot_3(0.0, 0.25, 0.0)
        rospy.sleep(1.5)

        # Detener el robot. 
        self.stop_robot_1()
        self.stop_robot_2()
        self.stop_robot_3()
        rospy.sleep(1)

        #Posciciones objetivo de los robots. 
        goal_pose_robot_1 = Pose2D()
        goal_pose_robot_1.x = self.pose_robot_1.x + 0.70
        goal_pose_robot_1.y = self.pose_robot_1.y -0.20

        goal_pose_robot_2 = Pose2D()
        goal_pose_robot_2.x = self.pose_robot_2.x + 0.70
        goal_pose_robot_2.y = self.pose_robot_2.y -0.20

        goal_pose_robot_3 = Pose2D()
        goal_pose_robot_3.x = self.pose_robot_3.x + 0.70
        goal_pose_robot_3.y = self.pose_robot_3.y -0.20

        # Variables para verificar si cada robot ha alcanzado su objetivo
        robot_1_reached = False
        robot_2_reached = False
        robot_3_reached = False

        # Bucle para mover los robots
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if not robot_1_reached:
                if abs(goal_pose_robot_1.x - self.pose_robot_1.x) >= VeraRobot.DISTANCE_TOLERANCE:
                    # Mover en el eje X
                    robot_1 = self.velocity_controller_robot_1()
                    self.update_command_robot_1(robot_1[0], 0.0, robot_1[1])
                else:
                    # Coordenada X alcanzada
                    self.stop_robot_1()
                    robot_1_reached = True

            if not robot_2_reached:
                if abs(goal_pose_robot_2.x - self.pose_robot_2.x) >= VeraRobot.DISTANCE_TOLERANCE:
                    # Mover en el eje X
                    robot_2 = self.velocity_controller_robot_2()
                    self.update_command_robot_2(1.2*robot_2[0], 0.0, robot_2[1])
                else:
                    # Coordenada X alcanzada
                    self.stop_robot_2()
                    robot_2_reached = True

            if not robot_3_reached:
                if abs(goal_pose_robot_3.x - self.pose_robot_3.x) >= VeraRobot.DISTANCE_TOLERANCE:
                    # Mover en el eje X
                    robot_3 = self.velocity_controller_robot_3()
                    self.update_command_robot_3(robot_3[0], 0.0, robot_3[1])
                else:
                    # Coordenada X alcanzada
                    self.stop_robot_3()
                    robot_3_reached = True

            # Verificar si todos los robots han alcanzado su objetivo
            if robot_1_reached or robot_2_reached or robot_3_reached:
                break

            rate.sleep()

        # Detener todos los robots por seguridad.
        self.stop_robot_1()
        self.stop_robot_2()
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_1()
        self.stop_robot_2()
        self.stop_robot_3()

        # Alejar el robot del objeto. 
        self.update_command_robot_2(-0.20, 0.0, 0.0)
        rospy.sleep(2)
        self.stop_robot_2()
        rospy.sleep(1)

        # Mover el objeto hacia la deracha. 
        self.update_command_robot_1(0.0, 0.35, 0.0)
        self.update_command_robot_3(0.0, 0.50, 0.0)
        rospy.sleep(2)

        # Detener todos los robots por seguridad
        self.stop_robot_1()
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_1()
        self.stop_robot_3()

        # Posicionar el objeto verticalmente
        #self.orient_to_angle_robot_1_3(0)

        # Detener todos los robots por seguridad
        self.stop_robot_1()
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_1()
        self.stop_robot_3()

        
        # -------------------------------------------------------------- SEGUNDO CICLO -------------------------------------------------
        # Mandar el primer robot a mover el objeto         

        # Variables para verificar si cada robot ha alcanzado su objetivo
        robot_1_reached = False
        robot_2_reached = False
        robot_3_reached = False

        self.send_goals_to_object(2)

        # Mover el objeto hacia adelante con todos los robots.
        while not rospy.is_shutdown():
            if self.result_data_robot_2.status.status == 3:
                print("El robot 2 ha llegado a su objetivo")
                self.stop_robot_2()
                rospy.sleep(1.5)
                self.update_command_robot_2(0.20, 0.0, 0.0)
                rospy.sleep(1.5)
                self.stop_robot_2()
                rospy.sleep(1.5)
                break

            rate.sleep()

        while not rospy.is_shutdown():
                if not robot_1_reached:
                    if abs(goal_pose_robot_1.x - self.pose_robot_1.x + 0.70) >= VeraRobot.DISTANCE_TOLERANCE:
                        # Mover en el eje X
                        robot_1 = self.velocity_controller_robot_1()
                        self.update_command_robot_1(robot_1[0], 0.0, robot_1[1])
                    else:
                        # Coordenada X alcanzada
                        self.stop_robot_1()
                        robot_1_reached = True

                if not robot_2_reached:
                    if abs(goal_pose_robot_2.x - self.pose_robot_2.x + 0.70) >= VeraRobot.DISTANCE_TOLERANCE:
                        # Mover en el eje X
                        robot_2 = self.velocity_controller_robot_2()
                        self.update_command_robot_2(robot_2[0], 0.0, robot_2[1])
                    else:
                        # Coordenada X alcanzada
                        self.stop_robot_2()
                        robot_2_reached = True

                if not robot_3_reached:
                    if abs(goal_pose_robot_3.x - self.pose_robot_3.x + 0.70) >= VeraRobot.DISTANCE_TOLERANCE:
                        # Mover en el eje X
                        robot_3 = self.velocity_controller_robot_3()
                        self.update_command_robot_3(robot_3[0], 0.0, robot_3[1])
                    else:
                        # Coordenada X alcanzada
                        self.stop_robot_3()
                        robot_3_reached = True

                # Verificar si todos los robots han alcanzado su objetivo
                if robot_1_reached or robot_2_reached or robot_3_reached:
                    break

                rate.sleep()

        # Detener todos los robots por seguridad.
        self.stop_robot_1()
        self.stop_robot_2()
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_1()
        self.stop_robot_2()
        self.stop_robot_3()

        # Alejar el robot del objeto. 
        self.update_command_robot_2(-0.20, 0.0, 0.0)
        rospy.sleep(2)
        self.stop_robot_2()
        rospy.sleep(1)

        # Mover el objeto hacia la deracha. 
        self.update_command_robot_1(0.0, -0.50, 0.0)
        self.update_command_robot_3(0.0, -0.35, 0.0)
        rospy.sleep(3)

        # Detener todos los robots por seguridad
        self.stop_robot_1()
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_1()
        self.stop_robot_3()

        # Posicionar el objeto verticalmente
        #self.orient_to_angle_robot_1_3(0)

        # Detener todos los robots por seguridad
        self.stop_robot_1()
        self.stop_robot_3()
        rospy.sleep(1)
        self.stop_robot_1()
        self.stop_robot_3()

    def run(self):
        rate = rospy.Rate(50)

        command = input("Ingrese 'go' para enviar las posiciones o 'exit' para salir: ")
        if command == "go":
            self.send_goals_to_object(1)
            self.send_goals_to_object(2)
            self.send_goals_to_object(3)

        while not rospy.is_shutdown():
            if self.result_data_robot_1.status.status == 3 and self.result_data_robot_2.status.status == 3 and self.result_data_robot_3.status.status == 3:
                print("todos los robots han lleagdo a su objetivo")
                
                self.result_data_robot_1.status.status = None
                self.result_data_robot_2.status.status = None
                self.result_data_robot_3.status.status = None

                self.stop_robot_1()
                self.stop_robot_2()
                self.stop_robot_3()

                while not rospy.is_shutdown():
                    command = input("Ingrese 'again' para enviar las posiciones o 'next' para seguir: ")
                    if command == "next":
                        break
                    elif command == "again":
                        rospy.sleep(1)
                        self.send_goals_to_object(1)
                        self.send_goals_to_object(2)
                        self.send_goals_to_object(3)
                        if self.result_data_robot_1.status.status == 3 and self.result_data_robot_2.status.status == 3 and self.result_data_robot_3.status.status == 3:
                            print("todos los robots han lleagdo a su objetivo")
                            self.stop_robot_1()
                            self.stop_robot_2()
                            self.stop_robot_3()
                            break
                    rate.sleep()

                self.move_object()
                break
            rate.sleep()
                
    def move_forward(self, speed, duration):
        self.update_command_robot_1(speed, 0, 0)
        self.update_command_robot_2(speed, 0, 0)
        self.update_command_robot_3(speed, 0, 0)
        rospy.sleep(duration)
        self.stop_robot_1()
        self.stop_robot_2()
        self.stop_robot_3()

    def get_current_odom_robot_1(self):
        print("Roboto 1:")
        if self.odom_data_robot_1:
            return self.odom_data_robot_1.pose.pose.position, self.odom_data_robot_1.pose.pose.orientation 
        else:
            return None
        
    def get_current_odom_robot_2(self):
        print("Roboto 2:")
        if self.odom_data_robot_2:
            return self.odom_data_robot_2.pose.pose.position, self.odom_data_robot_2.pose.pose.orientation
            
        else:
            return None
        
    def get_current_odom_robot_3(self):
        print("Robot 3:")
        if self.odom_data_robot_3:
            return self.odom_data_robot_3.pose.pose.position, self.odom_data_robot_3.pose.pose.orientation 
        else:
            return None    
    
    def get_current_poses(self):
        print("Robot_1")
        print(self.pose_robot_1.x) 
        print(self.pose_robot_1.y)
        print(self.pose_robot_1.theta)

        print("Robot_2")
        print(self.pose_robot_2.x) 
        print(self.pose_robot_2.y)
        print(self.pose_robot_2.theta)

        print("Robot_3")
        print(self.pose_robot_3.x) 
        print(self.pose_robot_3.y)
        print(self.pose_robot_3.theta)

if __name__ == '__main__':
    
    controller = VeraRobot()

    try:
        # Inicializar robots y esperar por datos de odometría
        controller.wait_for_subscribers_cmd_vel()
        controller.wait_for_subscribers_move_base()

        controller.wait_for_odom_data_robot_1()
        controller.wait_for_odom_data_robot_2()
        controller.wait_for_odom_data_robot_3()
        controller.wait_for_odom_data_objeto()

        rospy.loginfo("Todos los robots están listos.")

        # Iniciar movimiento de los robots (podría ser en hilos separados si se requiere simultaneidad)
        controller.run()

        
    except rospy.ROSInterruptException:
        pass