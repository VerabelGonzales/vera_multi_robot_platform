#!/usr/bin/env python

import threading
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class VeraRobot:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.vel_publisher = rospy.Publisher(f"/robot_{robot_id}/cmd_vel", Twist, queue_size=10)
        self.ir_subscriber = rospy.Subscriber(f"/robot_{robot_id}/ir_distances", Float32MultiArray, self.ir_callback)
        self.odom_subscriber = rospy.Subscriber(f"/robot_{robot_id}/odom", Odometry, self.odom_callback)
        self.ir_distances = []
        self.command = Twist()
        self.condition = threading.Condition()
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def wait_for_subscribers(self):
        while not rospy.is_shutdown() and self.vel_publisher.get_num_connections() == 0:
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            raise Exception("Shutdown requested before subscribers connected")

    def odom_callback(self, msg):
        pass

    def ir_callback(self, msg):
        self.ir_distances = [round(distance, 2) for distance in msg.data]

    def update_command(self, linear, angular):
        with self.condition:
            self.command.linear.x = linear
            self.command.angular.z = angular
            self.condition.notify()

    def publish_loop(self):
        rate = rospy.Rate(20)  # 10 Hz
        while not rospy.is_shutdown():
            with self.condition:
                self.condition.wait()  # Esperar hasta que se actualice el comando
                self.vel_publisher.publish(self.command)
            rate.sleep()

# Código principal
if __name__ == "__main__":

    rospy.init_node('vera_robot_controller')

    robot1 = VeraRobot(1)
    robot2 = VeraRobot(2)
    robot3 = VeraRobot(3)

    # Esperar a que los suscriptores se conecten
    robot1.wait_for_subscribers()
    robot2.wait_for_subscribers()
    robot3.wait_for_subscribers()

    linear_speed = 0.12
    move_time = 14# segundos

    # Actualizar comando de velocidad
    robot1.update_command(linear_speed, 0)
    robot2.update_command(linear_speed, 0)
    robot3.update_command(linear_speed, 0)

    rospy.sleep(move_time)

    # Detener los robots
    robot1.update_command(0, 0)
    robot2.update_command(0, 0)
    robot3.update_command(0, 0)
