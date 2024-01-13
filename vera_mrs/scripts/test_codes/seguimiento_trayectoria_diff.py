#!/usr/bin/env python3

import rospy, math
from geometry_msgs.msg import Twist, Pose2D #To publish velocities and tracking errors using the Pose2D message type
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi
import numpy as np

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Initialize variables (robot posture) and the offset of the outside point
x = 0
y = 0
yaw = 0
l = 0.1

vel_msg = Twist()
error_msg = Pose2D()
t = 0.0; t0 = 0.0; V_max = 0.22; W_max = 2.0; #Timer, initial time, maximum velocities [m/s, rad/s], respectively, for a burger type
T = 100.0; k = 0.15; #Trajectory period, controller gains kx = ky = k
ex = 0.0; ey = 0.0; V = 0.0; W = 0.0

def getKey(): #Function to use keyboard events on Linux
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def odomCallback(msg): #Callback function to get the robot posture
	global x; global y; global yaw
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	#Operations to convert from quaternion to Euler angles (and vice-versa)
	quater = msg.pose.pose.orientation
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(roll, pitch, yaw) = euler_from_quaternion(quater_list) #Euler angles are given in radians
	#quat = quaternion_from_euler(roll, pitch,yaw); print quat
	
def velocity_controller(): #Function to generate the desired trajectory and to compute the signals control
	global ex, ey, Vx,Vy, W, Xd,Yd,Ad #Indicate that some variables are global to be used in the main_function
	#Desired trajectory: Lemniscate
	a = 1.0; b = 0.5; X0 = -0.55; Y0 = -0.022; w = 2*pi/T
	#Desired position on the plane
	Xd = X0+a*sin(w*t)
	Yd = Y0+b*sin(2*w*t)
	Ad = 0

	#Corresponding time derivatives
	Xdp = a*w*cos(w*t)
	Ydp = 2*b*w*cos(2*w*t)
	Adp = 0
      
	ex = Xd - x 
	ey = Yd - y
	ea = Ad - yaw
		
	R = np.array([[np.cos(yaw), -np.sin(yaw), 0],
					[np.sin(yaw),  np.cos(yaw), 0],
					[0,  0, 1]])
	
	R_inv = np.linalg.inv(R)
	u = np.array([Xdp + k * ex, Ydp + k * ey, Adp + k*ea])
	U = np.dot(R_inv, u)
	Vx = U[0]
	Vy = U[1]
	W = U[2]

def main_function():
	rospy.init_node('diff_robot_controller', anonymous=False) #Initialize the node
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0
	
	vel_pub = rospy.Publisher('robot_3/cmd_vel', Twist, queue_size=10) #To publish in the topic
	rospy.Subscriber('robot_3/odom',Odometry, odomCallback) #To subscribe to the topic

	error_pub = rospy.Publisher('/tracking_errors', Pose2D, queue_size=10);

	file_obj = open("data_py.txt","w+") #If only the filename is given
	#the program will create the file in "/home/user/"
	
	#Important: Due to a differential type mobile robot is used, the following fields are ignored
	vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0;
	
	error_msg.theta = 0 #Since the orientation angle is a sub-actuated state, this field is assigned equal to zero

	
	print("To finish this node and to stop the robot, please press 'ctrl+C' or 'q' key\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n")
	
	global t, t0 #t and t0 are global to be used in velocity_controller
	t0 = rospy.Time.now().to_sec()
	
	while(1):
		t = rospy.Time.now().to_sec()-t0 #Compute the controller time    
		velocity_controller() #Compute the control signals
		
		vel_msg.linear.x = Vx
		vel_msg.linear.y = Vy
		vel_msg.angular.z = W
		vel_pub.publish(vel_msg); #Publish the control signals
	
		error_msg.x = ex; error_msg.y = ey; #Assign the tracking errors
		error_pub.publish(error_msg); #Publish the tracking errors

		#Convertion from float to string (to write in a file)
		text_t = "{:.3f}".format(t) 
		text_Xd = "{:.3f}".format(Xd); text_Yd = "{:.3f}".format(Yd)
		text_x = "{:.3f}".format(x); text_y = "{:.3f}".format(y)
		text_V = "{:.3f}".format(V); text_W = "{:.3f}".format(W)
		text_line = text_t+"\t"+text_Xd+"\t"+text_Yd+"\t"+text_x+"\t"+text_y+"\t"+text_V+"\t"+text_W+"\n"
		#print (text_line)

		file_obj.write(text_line)
		
		if counter == 25:
			rospy.loginfo("t: %.2f\tex: %.3f\tey: %.3f\tV: %.3f\tW: %.2f\n", t,ex,ey,V,W) #Print in terminal some variables
			counter = 0 #Reset the counter
		else:
			counter += 1
		
		key = getKey()
		if(key == 'q' or key == '\x03'): #\x03 is ctrl+C
			break
		
		rate.sleep() #spinOnce() function does not exist in python
	
	file_obj.close()

	#Stop the mobile robot
	vel_msg.linear.x = 0.0; vel_msg.angular.z = 0.0
	vel_pub.publish(vel_msg)
	
	print("Robot stops, but simulation keeps running\n")


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        main_function() #Execute the function
    except rospy.ROSInterruptException:
        pass

    #if os.name != 'nt':
    #   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)