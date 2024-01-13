#!/usr/bin/env python3

import rospy
import roslib
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
from std_msgs.msg import Int16MultiArray

class OmniTf:

    def __init__(self):
        rospy.init_node("wheel_odometry_robot_1")
        self.nodename = rospy.get_name()
        rospy.loginfo("Nodo %s Iniciado" % self.nodename)
        
        self.rate = rospy.get_param('~rate', 10.0)
        self.ticks_meter = float(rospy.get_param('ticks_meter', 29))
        self.wheel_radius = float(rospy.get_param('~wheel_radius', 0.03))
        self.l = float(rospy.get_param('~base_length', 0.07))
        self.r = float(rospy.get_param('~base_width', 0.093))
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        self.enc_lf = None
        self.enc_rf = None
        self.enc_lr = None
        self.enc_rr = None
        
        self.lf = 0
        self.rf = 0
        self.lr = 0
        self.rr = 0

        self.lfmult = 0
        self.rfmult = 0
        self.lrmult = 0
        self.rrmult = 0

        self.prev_lfencoder = 0
        self.prev_rfencoder = 0
        self.prev_lrencoder = 0
        self.prev_rrencoder = 0

        self.th = 0
        self.x = 0
        self.y = 0

        self.then = rospy.Time.now()
        
        rospy.Subscriber("robot_1/wheel_ticks", Int16MultiArray, self.ticksCallback)

        self.odomPub = rospy.Publisher("robot_1/odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            if self.enc_lf == None:
                d_lf, d_rf, d_lr, d_rr = 0, 0, 0, 0
            else:
                d_lf = (self.lf - self.enc_lf) / self.ticks_meter
                d_rf = (self.rf - self.enc_rf) / self.ticks_meter
                d_lr = (self.lr - self.enc_lr) / self.ticks_meter
                d_rr = (self.rr - self.enc_rr) / self.ticks_meter
            
            self.enc_lf = self.lf
            self.enc_rf = self.rf
            self.enc_lr = self.lr
            self.enc_rr = self.rr
            
            Vx = self.wheel_radius/4 * (d_lf + d_rf + d_lr + d_rr)
            Vy = self.wheel_radius/4 * (-d_lf + d_rf - d_lr + d_rr)
            Vtheta = self.wheel_radius/(4*(self.l+self.r)) * (-d_lf + d_rf + d_lr - d_rr)

            self.x += Vx * elapsed
            self.y += Vy * elapsed
            self.th += Vtheta * elapsed

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = Vx
            odom.twist.twist.linear.y = Vy
            odom.twist.twist.angular.z = Vtheta
            self.odomPub.publish(odom)   

    # Callbacks for all wheels

    def ticksCallback(self, msg):
        self.encoderCallback(msg.data[0], 'lf')
        self.encoderCallback(msg.data[1], 'rf')
        self.encoderCallback(msg.data[2], 'lr')
        self.encoderCallback(msg.data[3], 'rr')

    def encoderCallback(self, enc, wheel):
        prev_encoder = getattr(self, f'prev_{wheel}encoder')
        mult = getattr(self, f'{wheel}mult')
        
        if (enc < self.encoder_low_wrap and prev_encoder > self.encoder_high_wrap):
            mult += 1
        if (enc > self.encoder_high_wrap and prev_encoder < self.encoder_low_wrap):
            mult -= 1

        setattr(self, wheel, 1.0 * (enc + mult * (self.encoder_max - self.encoder_min)))
        setattr(self, f'prev_{wheel}encoder', enc)

if __name__ == '__main__':
    try:
        omniTf = OmniTf()
        omniTf.spin()
    except rospy.ROSInterruptException:
        pass
