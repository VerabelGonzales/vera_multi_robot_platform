#!/usr/bin/env python

import threading
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

class VeraRobot:
    def __init__(self, robot_id):
        #ID, correspondiente a cada robot, puede ser 1,2,3
        self.robot_id = robot_id
        #Topico /cmd_vel, aqui publicamos las velocidades para los robots
        self.vel_publisher = rospy.Publisher(f"/robot_{robot_id}/cmd_vel", Twist, queue_size=10)
        #Topico /ir_distances, nos suscribimos a este topico para obtener las distancias detectada por cada sensor IR
        self.ir_subscriber = rospy.Subscriber(f"/robot_{robot_id}/ir_distances", Float32MultiArray, self.ir_callback)
        #Topico /odom, no suscribimos para obtener los datos de odometria de nuestros robots.  
        self.odom_subscriber = rospy.Subscriber(f"/robot_{robot_id}/odom", Odometry, self.odom_callback)
        
        # Vector, para almacenar las distancas IR
        self.ir_distances = []
        self.odom_data = None

        self.command = Twist()
        self.pose = Pose2D()
        self.rate = rospy.Rate(10)

        #Condiciones para que la publicacion de velocidades sea robusta
        self.condition = threading.Condition()
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
    
    #*****************************************************************************************************************#
    def wait_for_subscribers(self):
        try:
            while not rospy.is_shutdown() and self.vel_publisher.get_num_connections() == 0:
                rospy.sleep(0.1)
            if rospy.is_shutdown():
                raise rospy.exceptions.ROSInterruptException("Shutdown requested before subscribers connected")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(f"Error while waiting for subscribers: {e}")
    
    #****************************************************************************************************************#
    def wait_for_odom_data(self):
        while self.odom_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        return
    
    #****************************************************************************************************************#
    def odom_callback(self, msg):
        self.odom_data = msg
    
    #****************************************************************************************************************#
    def ir_callback(self, msg):
        try:
            self.ir_distances = [round(distance, 2) for distance in msg.data]
        except Exception as e:
            rospy.logerr(f"Error in ir_callback: {e}")
    
    #****************************************************************************************************************#
    def update_command(self, linear_x, linear_y, angular_z):
        with self.condition:
            self.command.linear.x = linear_x
            self.command.linear.y = linear_y
            self.command.angular.z = angular_z
            self.condition.notify()
    
    #****************************************************************************************************************#
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
    
    #****************************************************************************************************************#
    def move_forward(self, speed, duration):
        self.update_command(speed, 0, 0)
        rospy.sleep(duration)
        self.stop()
    
    #****************************************************************************************************************#
    def move_lateral(self, speed, duration):
        self.update_command(0, speed, 0)
        rospy.sleep(duration)
        self.stop()
    
    #****************************************************************************************************************#
    def rotate(self, angular_speed, duration):
        self.update_command(0, 0, angular_speed)
        rospy.sleep(duration)
        self.stop()
    
    #****************************************************************************************************************#
    def stop(self):
        self.update_command(0, 0, 0)
        rospy.sleep(1)
        self.update_command(0, 0, 0)
        rospy.sleep(1)
        self.update_command(0, 0, 0)
        rospy.sleep(1)
        self.update_command(0, 0, 0)
        rospy.sleep(1)
    
    #****************************************************************************************************************#
    def get_current_position(self):
        if self.odom_data:
            return self.odom_data.pose.pose.position
        else:
            return None
    
    #****************************************************************************************************************#  
    def get_current_orientation(self):
        if self.odom_data:
            return self.odom_data.pose.pose.orientation
        else:
            return None
    
    #****************************************************************************************************************# 
    def get_yaw(self):
        if self.odom_data:
            orientation_q = self.odom_data.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            return yaw
        else:
            return 0

