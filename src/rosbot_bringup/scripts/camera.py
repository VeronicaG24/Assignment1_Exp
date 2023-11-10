#!/usr/bin/env python

# Python libraries
import sys
import time

# NumPy and SciPy
import numpy as np
from scipy.ndimage import filters
import math

import imutils

# OpenCV
import cv2

# ROS libraries
import roslib
import rospy

# ROS Messages
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from gazebo_msgs.msg import LinkState
from std_msgs.msg import Int32
from gazebo_msgs.msg import LinkStates

# Constants
pixel_limit = 170 # limit for stopping the robot
width_camera = 320 # dimension of the camera
lin_vel_move = 0.2 # linear velocity
ang_vel_move = 0.5 # angular velocity
no_vel_move = 0.0 # stop velocity
pixel_thr = 15 # threshold in pixels for allignment 

# Class for image features
class image_feature:
    def __init__(self, mode_param):
        '''Initialize ROS publisher, ROS subscriber'''
        rospy.init_node('image_feature', anonymous=True)

        # Topic where we publish velocity commands
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Publisher for the JointState message
        self.joint_state_pub = rospy.Publisher("/robot_exp/camera_velocity_controller/command", Float64, queue_size=1)

        # Subscriber for camera image
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.move_callback, queue_size=1)

        # Subscriber for marker ID
        self.subscriber = rospy.Subscriber("/id_publisher", Int32, self.id_callback, queue_size=1)

        # Subscriber for acknowledgment from the camera
        self.subscriber_ack = rospy.Subscriber("/ack_camera", Bool, self.ack_callback, queue_size=1)

        # Subscriber for robot/camera pose and orientation
        self.subscriber_pose = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_callback, queue_size=1)

        # Subscriber for the marker center
        self.marker_center_sub = rospy.Subscriber("/marker_point", Point, self.marker_center_callback, queue_size=1)

        # Subscriber for the pixel side of the marker
        self.pixel_side_sub = rospy.Subscriber("/pixel_side_marker", Float64, self.pixel_callback, queue_size=1)

        # Retrieve marker list from parameter server
        self.marker_list = rospy.get_param('/marker_publisher/marker_list')
        
        # initialize variables
        self.marker_center_x = 0.0 # center x
        self.marker_center_y = 0.0 # center y
        self.marker_id = 0 # marker id
        self.current_pixel_side = 0.0 # pixel side of the marker
        
        self.orientation_robot = 0.0 # orientation of the robot
        self.orientation_camera = 0.0 # orientation of the camera

        # Control gains
        self.Kl = 0.015
        self.Ka = 4.0
        
        # Operating mode (0 for fixed camera, 1 for mobile camera)
        self.mode = mode_param
        
        self.allineato = False # USATO PER MODE 1 --> DA TOGLIERE PROBABILMENTE
        
    def pose_callback(self, data):
        # Extract orientation information from robot and camera
        self.orientation_robot = (data.pose[9].orientation.x, data.pose[9].orientation.y, data.pose[9].orientation.w)
        self.orientation_camera = (data.pose[10].orientation.x, data.pose[10].orientation.y, data.pose[10].orientation.w)

    def move_callback(self, ros_data):
        # Check if the marker list is empty
        if not self.marker_list:
            # Send a message to close marker_publisher on the topic where we write the marker number
            rospy.signal_shutdown("All markers reached")
            return

        if self.mode == 0:
            # operation mode: camera fixed 
            if self.marker_id == self.marker_list[0]:
                print("MARKER FOUND!")

                # Compute error
                error = abs(self.marker_center_x - width_camera)

                if self.current_pixel_side > pixel_limit:
                    # stop the robot when the marker is reached
                    print("MARKER REACHED: " + str(self.marker_id))
                    cmd_vel = Twist()
                    cmd_vel.linear.x = no_vel_move
                    cmd_vel.angular.z = no_vel_move
                    self.vel_pub.publish(cmd_vel)

                    # Remove the first element from the list to move to the next marker
                    self.marker_list.pop(0)
                    rospy.set_param('/marker_publisher/marker_list', self.marker_list)

                elif error < pixel_thr:
                    # robot alligned with the center of the marker
                    print("ALIGNED!")
                    cmd_vel = Twist()
                    cmd_vel.linear.x = lin_vel_move
                    cmd_vel.angular.z = no_vel_move
                    self.vel_pub.publish(cmd_vel)

                else:
                    # allign the robot with the center of the marker
                    cmd_vel = Twist()
                    cmd_vel.linear.x = lin_vel_move
                    if self.marker_center_x < width_camera:
                        cmd_vel.angular.z = ang_vel_move
                    else:
                        cmd_vel.angular.z = -lin_vel_move
                    
                    self.vel_pub.publish(cmd_vel)

            else:
                # looking for the marker
                cmd_vel = Twist()
                cmd_vel.linear.x = no_vel_move
                cmd_vel.angular.z = ang_vel_move
                self.vel_pub.publish(cmd_vel)

        elif self.mode == 1:
            # operation mode: camera fixed 
            if self.marker_id == self.marker_list[0]:
                error = abs(self.marker_center_x - width_camera)

                if error < pixel_thr:
                    self.allineato = True
                    print("I AM HERE! if ERROR")
                    vel_camera = Float64()
                    vel_camera.data = no_vel_move
                    self.joint_state_pub.publish(vel_camera)
                    
                    orientation_diff_x = abs(self.orientation_camera[0] - self.orientation_robot[0])
                    orientation_diff_y = abs(self.orientation_camera[1] - self.orientation_robot[1])
                    orientation_diff_w = abs(self.orientation_camera[2] - self.orientation_robot[2])
                    
                    if orientation_diff_x < 0.05 and orientation_diff_y < 0.05 and orientation_diff_w < 0.5:
                        print("ROBOT ALIGNED!")
                        print(orientation_diff_x)
                        print(orientation_diff_y)
                        print(orientation_diff_w)
                        vel_camera = Float64()
                        cmd_vel = Twist()
                        cmd_vel.linear.x = lin_vel_move
                        cmd_vel.angular.z = no_vel_move
                        vel_camera.data = no_vel_move
                        self.vel_pub.publish(cmd_vel)
                        self.joint_state_pub.publish(vel_camera)
                        allineato = False
                        
                    else:
                        vel_camera = Float64()
                        cmd_vel = Twist()
                        print("I NEED TO ALIGN THE ROBOT!")
                        print(orientation_diff_x)
                        print(orientation_diff_y)
                        print(orientation_diff_w)
                        cmd_vel.angular.z = ang_vel_move
                        vel_camera.data = -ang_vel_move
                        self.vel_pub.publish(cmd_vel)
                        self.joint_state_pub.publish(vel_camera)
                else:
                    vel_camera = Float64()
                    print("I AM HERE! else ERROR")
                    vel_camera.data = ang_vel_move
                    self.joint_state_pub.publish(vel_camera)
            else:
                if self.allineato == False:
                    vel_camera = Float64()
                    print("I AM HERE! else MARKER")
                    vel_camera.data = ang_vel_move
                    self.joint_state_pub.publish(vel_camera)

    def id_callback(self, data):
        # Callback function for marker ID
        self.marker_id = data.data

    def ack_callback(self, ack):
        # Callback function for acknowledgment from the camera
        self.ack_data = ack.data

    def marker_center_callback(self, data):
        # Callback function for marker center
        self.marker_center_x = data.x
        self.marker_center_y = data.y

    def pixel_callback(self, data):
        # Callback function for pixel side of the marker
        self.current_pixel_side = data.data

def main(args):
    '''Initializes and cleans up ROS node'''

    # Ask the user to enter 0 or 1 for choosing the mode operation
    while True:
        mode_input = input("Enter 0 (for camera fixed) or 1 (for mobile camera): ")
        if mode_input in ['0', '1']:
            break
        else:
            print("Enter a valid value (0 or 1)")

    # Convert the input to an integer
    mode_param = int(mode_input)

    ic = image_feature(mode_param)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

