#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import math

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState  # Import JointState message
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from gazebo_msgs.msg import LinkState
from std_msgs.msg import Int32




class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        
        # topic where we publish
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Aggiunto il publisher per il messaggio JointState
        self.joint_state_pub = rospy.Publisher("/robot_exp/camera_velocity_controller/command", Float64, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.move_callback, queue_size=1)
        
        self.subscriber = rospy.Subscriber("/id_publisher", Int32, self.id_callback, queue_size=1)

        
        self.subscriber_ack = rospy.Subscriber("/ack_camera", Bool, self.ack_callback, queue_size=1)

        # Subscriber for the marker center
        self.marker_center_sub = rospy.Subscriber("/marker_point", Point, self.marker_center_callback, queue_size=1)

        self.pixel_side_sub = rospy.Subscriber("/pixel_side_marker", Float64, self.pixel_callback, queue_size=1)

        self.marker_list = [11, 12, 13, 15]
        self.marker_center_x = 0.0
        self.marker_center_y = 0.0
        self.marker_id = 0
        self.current_pixel_side = 0.0
        #elf.error = 0.0
        
        # Control gains
        self.Kl = 0.015
        self.Ka = 4.0
        
        

        

    

    def move_callback(self, ros_data):
         # controllo se la lista è vuota e se cos' fosse il programma finisce ( o così dovrebbe fare)
             if not self.marker_list:
                 rospy.signal_shutdown("All markers reached")
                 return
                 
             if self.marker_id == self.marker_list[0]:
             
                 rospy.loginfo("MARKER TROVATO!")
                 
                 #computer error
                 error = abs(self.marker_center_x - 320)
                 #max_error = 16
                 
                 if self.current_pixel_side > 170:
                     rospy.loginfo("MARKER {} RAGGIUNTO!".format(self.marker_id))
                     cmd_vel = Twist()
                     cmd_vel.linear.x = 0.0
                     cmd_vel.angular.z = 0.0
                     self.vel_pub.publish(cmd_vel)
                     
                     self.marker_list.pop(0) #levo il primo elemento della lista così passo alla ricerca del marker successivo
                     
                     
                 
                 elif error < 15:
                     rospy.loginfo("ALLINEATOOOOOOOOOOO!")
                     
                     cmd_vel = Twist()
                     cmd_vel.linear.x = 0.4#self.Kl * error
                     cmd_vel.angular.z = 0.0
                     self.vel_pub.publish(cmd_vel)
                     
                 else:
                     cmd_vel = Twist()
                     #error = min(error, max_error)
                     cmd_vel.linear.x = 0.2#self.Kl * error
                     if self.marker_center_x < 320:
                         cmd_vel.angular.z = 0.2#self.Ka * error
                     else:
                         cmd_vel.angular.z = -0.2 #-self.Ka * error
                     self.vel_pub.publish(cmd_vel)
                 
                     
                 
             else: 
                 cmd_vel = Twist()
                 cmd_vel.linear.x = 0.0
                 cmd_vel.angular.z = 0.5
                 self.vel_pub.publish(cmd_vel)
             
        
         
         
         
         
         
    
    
    def id_callback(self, data):
        self.marker_id = data.data
        
    

    def ack_callback(self, ack):
        self.ack_data = ack.data
     
    def marker_center_callback(self, data):
        self.marker_center_x = data.x
        self.marker_center_y = data.y
        

    def pixel_callback(self, data):
        self.current_pixel_side = data.data
        print("PIXEL: ", self.current_pixel_side)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

