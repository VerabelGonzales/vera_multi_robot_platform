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
        #self.ir_subscriber_robot_2 = rospy.Subscriber("robot_2/ir_distances", Float32MultiArray, self.ir_callback_robot_2)
        #self.ir_subscriber_robot_3 = rospy.Subscriber("robot_3/ir_distances", Float32MultiArray, self.ir_callback_robot_3)


    # Callbacks para cada robot
    def ir_callback_robot_1(self, data):
        print(round(data.data[2],2))

    #def ir_callback_robot_2(self, data):
    #    self.publish_lidar_data_2(data, self.lidar_publisher_robot_2)

    #def ir_callback_robot_3(self, data):
    #    self.publish_lidar_data_3(data, self.lidar_publisher_robot_3)


if __name__ == '__main__':
    try:
        node = IRtoLidarNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
