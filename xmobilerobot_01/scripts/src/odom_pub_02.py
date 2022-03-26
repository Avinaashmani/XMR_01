#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16

from math import sin, cos, atan

import numpy as np
import cv2 as cv

#x = 0.0
#y = 0.0
#th = 0.0

# Odometry is Done Using the Encoder Values

class Odometry_Compute:
    # global x, y, th
    def __init__(self):
        rospy.init_node ('Odom_Node', anonymous=False)
        rospy.loginfo ("Welcome, Node has Begun")
        
        self.stamp = rospy.Time.now()
        self.frame_id = 'odom'
        self.child_string_id = 'base_link'
        self.base_link_id = 'base_link'

        self.last_time = rospy.Time.now()
        # self.current_time = rospy.Time.now()

        self.rate = 10.0
        self.encoder_val =  22275
        self.base_width = 0.0285
        self.wheel_radius = 0.0325
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        self.th = 0.0
        self.x = 0.0
        self.y = 0.0
        self.vth = 0.0
        self.vx = 0.0
        rospy.Subscriber ('/encoder_data/right', Int16, self.r_wheel_cb)
        rospy.Subscriber ('/encoder_data/left' , Int16, self.l_wheel_cb)
        self.odomPub = rospy.Publisher ('odom', Odometry, queue_size=10)
        self.odom_broadcast = TransformBroadcaster()

    def ros_spin(self):
        r = rospy.Rate (self.rate)
        while not rospy.is_shutdown():
            self.compute()
            r.sleep()

    def compute (self):

        self.current_time = rospy.Time.now()
        self.vx = (self.left_speed + self.right_speed)/ 2
        self.vth = (self.left_speed - self.right_speed) / self.base_width
        self.dt = (self.current_time - self.last_time)
        self.delta_x = (self.vx * cos(self.th)) / 100
        self.delta_y = (self.vx * sin(self.th)) / 100
        self.delta_th = self.vth / 100
        self.x = self.x + self.delta_x
        self.y = self.y + self.delta_y
        self.th = self.th + self.delta_th

        # PUBLISHING THE VALUES ON THE TOPICS #

        quternion = Quaternion()
        quternion.x = 0.0
        quternion.y = 0.0
        quternion.z = sin(self.th / 2)
        quternion.w = cos(self.th / 2)

        self.odom_broadcast.sendTransform (
            (self.x, self.y, 0.0), 
            (quternion.x, quternion.y, quternion.z, quternion.w), 
            rospy.Time.now(),
            self.frame_id, 
            self.base_link_id)
        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_string_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quternion
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth
        self.odomPub.publish(odom)

    def r_wheel_cb (self, msg):

        self.right_speed = msg.data 
        
  
    def l_wheel_cb (self, msg):

        self.left_speed = msg.data 
        

if __name__ == '__main__':

    try:
        odom_comp = Odometry_Compute()
        odom_comp.ros_spin()
    except rospy.ROSInterruptException:
        pass
