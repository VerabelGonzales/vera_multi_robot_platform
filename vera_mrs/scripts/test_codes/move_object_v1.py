#!/usr/bin/env python
import rospy
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
import tf

class VeraRobot:
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


    def send_goals(self):
        if self.position_object is None or self.orientation_object is None:
            rospy.logwarn("Posición del objeto no recibida todavía.")
            return

        offset = 0.30
        goal_pose_1 = PoseStamped()
        goal_pose_1.header.stamp = rospy.Time.now()
        goal_pose_1.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame

        goal_pose_2 = PoseStamped()
        goal_pose_2.header.stamp = rospy.Time.now()
        goal_pose_2.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame
        
        goal_pose_3 = PoseStamped()
        goal_pose_3.header.stamp = rospy.Time.now()
        goal_pose_3.header.frame_id = "map"  # Asumiendo que todos usan el mismo frame

        # Calcular y enviar las metas para cada robot
        for robot_id in [1, 2, 3]:
            if robot_id == 1:
                goal_pose_1.pose.position.x = self.position_object.x 
                goal_pose_1.pose.position.y = self.position_object.y + offset
                goal_pose_1.pose.position.z = self.position_object.z
                goal_pose_1.pose.orientation = self.orientation_object
            elif robot_id == 2:
                goal_pose_2.pose.position.x = self.position_object.x - 0.35
                goal_pose_2.pose.position.y = self.position_object.y
                goal_pose_2.pose.position.z = self.position_object.z
                goal_pose_2.pose.orientation = self.orientation_object
            elif robot_id == 3:
                goal_pose_3.pose.position.x = self.position_object.x 
                goal_pose_3.pose.position.y = self.position_object.y - offset
                goal_pose_3.pose.position.z = self.position_object.z
                goal_pose_3.pose.orientation = self.orientation_object
                
        self.goal_robot_1_publisher.publish(goal_pose_1)
        self.goal_robot_2_publisher.publish(goal_pose_2)
        self.goal_robot_3_publisher.publish(goal_pose_3)

    def run(self):
        while not rospy.is_shutdown():
            command = input("Ingrese 'go' para enviar las posiciones o 'exit' para salir: ")
            if command == "exit":
                break
            elif command == "go":
                self.send_goals()

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

        rospy.loginfo("Todos los robots están listos.")

        # Iniciar movimiento de los robots (podría ser en hilos separados si se requiere simultaneidad)
        controller.run()

        
    except rospy.ROSInterruptException:
        pass