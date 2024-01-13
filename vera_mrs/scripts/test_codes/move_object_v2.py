#!/usr/bin/env python
import rospy
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
import tf
from math import pow, atan2, sqrt, degrees, pi
from actionlib_msgs.msg import GoalID

class VeraRobot:

    # Constante para el radio de la zona de seguridad
    SECURITY_ZONE_RADIUS = 0.20  # 25 cm

    DISTANCE_TOLERANCE = 0.15  # Velocidad lineal máxima
    ANGULAR_TOLERANCE = 0.15  # Velocidad angular máxima

    def __init__(self):
        rospy.init_node('vera_robots_controller', anonymous=True)

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
        # Publicador en el topico para cancelar la navegacion
        self.cancel_goal_robot_1 = rospy.Publisher("robot_1/move_base/cancel", GoalID, queue_size=10)
        self.cancel_goal_robot_2 = rospy.Publisher("robot_1/move_base/cancel", GoalID, queue_size=10)
        self.cancel_goal_robot_3 = rospy.Publisher("robot_1/move_base/cancel", GoalID, queue_size=10)
        
        # Última posición y orientación del objeto
        self.odom_data_robot_1 = None
        self.odom_data_robot_2 = None
        self.odom_data_robot_3 = None
        self.odom_data_object = None
        self.position_object = None
        self.orientation_object = None

        self.pose_robot_1 = Pose2D()
        self.pose_robot_2 = Pose2D()
        self.pose_robot_3 = Pose2D()
        self.pose_object = Pose2D()

        self.command_robot_1 = Twist()
        self.command_robot_2 = Twist()
        self.command_robot_3 = Twist()

        #Condiciones para que la publicacion de velocidades del robot 1
        self.condition_robot_1 = threading.Condition()
        self.publish_thread_robot_1 = threading.Thread(target=self.publish_loop_robot_1)
        self.publish_thread_robot_1.daemon = True
        self.publish_thread_robot_1.start()

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

        # Añadir la zona de seguridad para los robots
        self.security_zone_robot_1 = {'x': 0, 'y': 0, 'radius': VeraRobot.SECURITY_ZONE_RADIUS}
        self.security_zone_robot_2 = {'x': 0, 'y': 0, 'radius': VeraRobot.SECURITY_ZONE_RADIUS}
        self.security_zone_robot_3 = {'x': 0, 'y': 0, 'radius': VeraRobot.SECURITY_ZONE_RADIUS}

    def wait_for_subscribers_cmd_vel(self):
        try:
            while not rospy.is_shutdown() and self.vel_robot_1_publisher.get_num_connections() == 0 and self.vel_robot_2_publisher.get_num_connections() == 0 and self.vel_robot_3_publisher.get_num_connections() == 0:
                rospy.sleep(0.1)
            if rospy.is_shutdown():
                raise rospy.exceptions.ROSInterruptException("Shutdown requested before subscribers connected")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(f"Error while waiting for subscribers: {e}")
    
    def wait_for_subscribers_move_base_goal(self):
        try:
            while not rospy.is_shutdown() and self.goal_robot_1_publisher.get_num_connections() == 0 and self.goal_robot_2_publisher.get_num_connections() == 0 and self.goal_robot_3_publisher.get_num_connections() == 0:
                rospy.sleep(0.1)
            if rospy.is_shutdown():
                raise rospy.exceptions.ROSInterruptException("Shutdown requested before subscribers connected")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(f"Error while waiting for subscribers: {e}")
    
    def wait_for_subscribers_move_base_cancel_goal(self):
        try:
            while not rospy.is_shutdown() and self.cancel_goal_robot_1.get_num_connections() == 0 and self.cancel_goal_robot_2.get_num_connections() == 0 and self.cancel_goal_robot_3.get_num_connections() == 0:
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
        self.pose_robot_1.x = round(position.x, 4)
        self.pose_robot_1.y = round(position.y, 4)
        self.pose_robot_1.theta = yaw

        # Actualizar la zona de seguridad cada vez que la pose se actualiza
        self.update_security_zone_robot_1()
    
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
        self.pose_robot_2.x = round(position.x, 4)
        self.pose_robot_2.y = round(position.y, 4)
        self.pose_robot_2.theta = yaw

        # Actualizar la zona de seguridad cada vez que la pose se actualiza
        self.update_security_zone_robot_2()
    
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
        self.pose_robot_3.x = round(position.x, 4)
        self.pose_robot_3.y = round(position.y, 4)
        self.pose_robot_3.theta = yaw

        # Actualizar la zona de seguridad cada vez que la pose se actualiza
        self.update_security_zone_robot_3()
    
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

        self.pose_object.x = round(position.x, 4)
        self.pose_object.y = round(position.y, 4)
        self.pose_object.theta = yaw


    def update_security_zone_robot_1(self):
        #Actualiza la zona de seguridad del robot.
        self.security_zone_robot_1['x'] = self.pose_robot_1.x
        self.security_zone_robot_1['y'] = self.pose_robot_1.y
        # El radio se mantiene constante, pero podría actualizarse si es necesario
    
    def update_security_zone_robot_2(self):
        #Actualiza la zona de seguridad del robot.
        self.security_zone_robot_2['x'] = self.pose_robot_2.x
        self.security_zone_robot_2['y'] = self.pose_robot_2.y
        # El radio se mantiene constante, pero podría actualizarse si es necesario

    def update_security_zone_robot_3(self):
        #Actualiza la zona de seguridad del robot.
        self.security_zone_robot_3['x'] = self.pose_robot_3.x
        self.security_zone_robot_3['y'] = self.pose_robot_3.y
        # El radio se mantiene constante, pero podría actualizarse si es necesario


    def check_collision(self, robot_name1, robot_name2):
        """
        Verifica si hay una colisión o riesgo de colisión con otro robot.
        :param other_robot: Una instancia de VeraRobot con la que se va a verificar la colisión.
        :return: True si hay una colisión o riesgo de colisión, False en caso contrario.
        """
        robot_1_zone = getattr(self, f'security_zone_{robot_name1}')
        robot_2_zone = getattr(self, f'security_zone_{robot_name2}')

        distance = sqrt(pow((robot_1_zone['x'] - robot_2_zone['x']), 2) +
                        pow((robot_1_zone['y'] - robot_2_zone['y']), 2))

        #print(distance)
        # Comprobar si la distancia es menor que la suma de los radios de las zonas de seguridad
        if distance < (robot_1_zone['radius'] + robot_2_zone['radius']):
            return True  # Hay colisión o riesgo de colisión
        else:
            return False  # No hay colisión

    def calculate_relative_angle(self, current_robot_name, other_robot_name):
        
        current_robot_pose = getattr(self, f'pose_{current_robot_name}')
        other_robot_pose = getattr(self, f'pose_{other_robot_name}')

        # Diferencia en posición
        dx = other_robot_pose.x - current_robot_pose.x
        dy = other_robot_pose.y - current_robot_pose.y

        # Ángulo hacia el otro robot
        angle_to_other = atan2(dy, dx)

        # Ángulo relativo considerando la orientación del robot actual
        relative_angle = current_robot_pose.theta - angle_to_other

        # Normalizar el ángulo
        relative_angle = (relative_angle + pi) % (2 * pi) - pi

        return degrees(relative_angle)  # Convertir a grados para facilidad de interpretación


    def run(self):
        while not rospy.is_shutdown():
            command = input("Ingrese 'go' para enviar las posiciones o 'exit' para salir: ")
            if command == "exit":
                break
            elif command == "go":
                self.send_goals()

    def move_forward(self, speed, duration):
        self.update_command_robot_1(0, speed, 0)
        self.update_command_robot_2(0, speed, 0)
        self.update_command_robot_3(0, speed, 0)
        rospy.sleep(duration)
        self.stop_robot_1()
        self.stop_robot_2()
        self.stop_robot_3()

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


    def send_goals(self, robot_id):
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
            self.goal_1.pose.position.x = self.position_object.x + 0.15
            self.goal_1.pose.position.y = self.position_object.y + offset
            self.goal_1.pose.position.z = self.position_object.z
            self.goal_1.pose.orientation = self.orientation_object
            self.goal_robot_1_publisher.publish(self.goal_1)

        elif robot_id == 2:
            self.goal_2.pose.position.x = self.position_object.x - 0.15
            self.goal_2.pose.position.y = self.position_object.y
            self.goal_2.pose.position.z = self.position_object.z
            self.goal_2.pose.orientation = self.orientation_object
            self.goal_robot_2_publisher.publish(self.goal_2)

        elif robot_id == 3:
            self.goal_3.pose.position.x = self.position_object.x + 0.15
            self.goal_3.pose.position.y = self.position_object.y - offset
            self.goal_3.pose.position.z = self.position_object.z
            self.goal_3.pose.orientation = self.orientation_object
            self.goal_robot_3_publisher.publish(self.goal_3)

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

    def avoid_obstacles_robot_1(self, r_angle):

        flag = False

        if 0 < r_angle < 45 or 135 < r_angle < 180:
            # Mover hacia la derecha
            self.stop_robot_1()
            rospy.sleep(2)
            self.update_command_robot_1(0.0, 0.25, 0.0)
            rospy.sleep(1.8)
            self.stop_robot_1()
            rospy.sleep(2)

            flag = True
           
        elif -45 < r_angle < 0 or -135 < r_angle < -180:
            # Mover hacia la derecha
            # Mover hacia la derecha
            self.stop_robot_1()
            rospy.sleep(2)
            self.update_command_robot_1(0.0, -0.25, 0.0)
            rospy.sleep(1.8)
            self.stop_robot_1()
            rospy.sleep(2)

            flag = True
        
        # Determinar si el objetivo está adelante o atrás
        angle_to_goal = self.steering_angle(1)

        if flag == True or 45 <= r_angle <= 135 or -180 <= r_angle <= -45:

            if -pi/2 <= angle_to_goal <= pi/2:
                # Mover hacia adelante
                self.stop_robot_1()
                rospy.sleep(2)
                self.update_command_robot_1(0.20, 0.0, 0.0)
                rospy.sleep(2.2)
                self.stop_robot_1()
                rospy.sleep(2)

            else:
                # Mover hacia atras
                self.stop_robot_1()
                rospy.sleep(2)
                self.update_command_robot_1(-0.20, 0.0, 0.0)
                rospy.sleep(2.2)
                self.stop_robot_1()
                rospy.sleep(2)

    def steering_angle(self, robot_id):
        # Direccion del angulo.
        if robot_id == 1:
            return atan2(self.goal_1.pose.position.y - self.pose_robot_1.y, self.goal_1.pose.position.x - self.pose_robot_1.x)

        if robot_id == 2:
            return atan2(self.goal_2.pose.position.y - self.pose_robot_2.y, self.goal_2.pose.position.x - self.pose_robot_2.x)

        if robot_id == 3:
            return atan2(self.goal_3.pose.position.y - self.pose_robot_3.y, self.goal_3.pose.position.x - self.pose_robot_3.x)
    
    def navigation_on(self):

        self.send_goals(1)

        while not rospy.is_shutdown() and self.euclidean_distance(1) >= VeraRobot.DISTANCE_TOLERANCE:

            if self.check_collision('robot_1', 'robot_2'):
                # Crear y enviar un mensaje de cancelación
                cancel_msg = GoalID()  # Un mensaje vacío de GoalID cancelará la meta actual
                self.cancel_goal_robot_1.publish(cancel_msg)

                relative_angle = self.calculate_relative_angle('robot_1', 'robot_2')
                print(f"El Robot 1 se detuvo a {relative_angle} grados del Robot 2")

                self.avoid_obstacles_robot_1(relative_angle)

                self.send_goals(1)

            elif self.check_collision('robot_1', 'robot_3'):
                # Crear y enviar un mensaje de cancelación
                cancel_msg = GoalID()  # Un mensaje vacío de GoalID cancelará la meta actual
                self.cancel_goal_robot_1.publish(cancel_msg)

                relative_angle = self.calculate_relative_angle('robot_1', 'robot_3')
                print(f"El Robot 1 se detuvo a {relative_angle} grados del Robot 3")

                self.avoid_obstacles_robot_1(relative_angle)

                self.send_goals(1)
                    

if __name__ == '__main__':
    
    controller = VeraRobot()

    try:
        # Inicializar robots y esperar por datos de odometría
        controller.wait_for_subscribers_cmd_vel()
        controller.wait_for_subscribers_move_base_goal()
        controller.wait_for_subscribers_move_base_cancel_goal()

        controller.wait_for_odom_data_robot_1()
        controller.wait_for_odom_data_robot_2()
        controller.wait_for_odom_data_robot_3()

        rospy.loginfo("Todos los robots están listos.")

        # Iniciar movimiento de los robots (podría ser en hilos separados si se requiere simultaneidad)
        controller.navigation_on()

        
    except rospy.ROSInterruptException:
        pass