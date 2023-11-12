#!/usr/bin/env python

# Python libraries
import sys
import time

# NumPy and SciPy
import numpy as np
from scipy.ndimage import filters
import math

import imutils
from tf import transformations

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
pixel_thr = 18 # threshold in pixels for allignment 
yaw_thr = math.pi / 90  # +/- 2 degree allowed

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
        self.kp_d = 0.2
        self.kp_a = 3.0
        self.ub_d = 0.3
        self.ub_cr = 0.4 #upper bound camera rotation
        self.ub_br = 0.5 #upper bound base rotation
        
        self.yaw_robot = 0.0
        self.yaw_camera = 0.0
        
        
        # Operating mode (0 for fixed camera, 1 for mobile camera)
        self.mode = mode_param
        
        self.allineato = False # USATO PER MODE 1 --> DA TOGLIERE PROBABILMENTE
        
    def pose_callback(self, data):
        # Extract orientation information from robot and camera
        self.orientation_robot = (data.pose[9].orientation.x, data.pose[9].orientation.y, data.pose[9].orientation.z, data.pose[9].orientation.w)
        self.orientation_camera = (data.pose[10].orientation.x, data.pose[10].orientation.y, data.pose[10].orientation.z, data.pose[10].orientation.w)
        
       
        quaternion_robot = (
               self.orientation_robot[0],
               self.orientation_robot[1],
               self.orientation_robot[2],
               self.orientation_robot[3])
               
        euler_robot = transformations.euler_from_quaternion(quaternion_robot)
           
        self.yaw_robot = euler_robot[2]
       # print("YAW ROBOT", self.yaw_robot)
           
        quaternion_camera = (
               self.orientation_camera[0],
               self.orientation_camera[1],
               self.orientation_camera[2],
               self.orientation_camera[3])
               
        euler_camera = transformations.euler_from_quaternion(quaternion_camera)
           
        self.yaw_camera = euler_camera[2]
       # print("YAW CAMERA", self.yaw_camera)

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
                    # CONTROLLER for robot's alignment with marker
                    cmd_vel = Twist()
                    cmd_vel.linear.x = self.kp_d * error
                    if cmd_vel.linear.x > self.ub_d:
                        cmd_vel.linear.x = self.ub_d
                   
                    if self.marker_center_x < width_camera:
                        cmd_vel.angular.z = self.kp_a * error
                        if cmd_vel.angular.z > self.ub_d:
                            cmd_vel.angular.z = self.ub_d
                    else:
                        cmd_vel.angular.z = - self.kp_a * error
                        if cmd_vel.angular.z < - self.ub_d:
                            cmd_vel.angular.z = - self.ub_d
                    
                    self.vel_pub.publish(cmd_vel)

            else:
                # looking for the marker
                cmd_vel = Twist()
                cmd_vel.linear.x = no_vel_move
                cmd_vel.angular.z = ang_vel_move
                self.vel_pub.publish(cmd_vel)

        elif self.mode == 1:
        
           # operation mode: camera moving 
            if self.marker_id == self.marker_list[0]:
            
                print("MARKER FOUND: " + str(self.marker_id))
                
                yaw_error = self.normalize_angle(self.yaw_camera - self.yaw_robot)

                error = abs(self.marker_center_x - width_camera)
                
               

                if abs(yaw_error) <= yaw_thr or self.allineato == True:
                    # robot alligned with the center of the marker
                    print("ALLINEATOOOOOOOOOOOOOOOOOOO!")
                    print("YAWWWWWW: ", yaw_error)
                    print("ABSSSS: ", abs(yaw_error))
                    
                    
                    vel_camera = Float64()
                    
                    vel_camera.data = 0.0
                    self.joint_state_pub.publish(vel_camera)
                    
                    self.allineato = True
                    
                    if self.current_pixel_side > pixel_limit and self.allineato == True:
                        # stop the robot when the marker is reached
                        print("MARKER REACHED: " + str(self.marker_id))
                        cmd_vel = Twist()
                        cmd_vel.linear.x = no_vel_move
                        cmd_vel.angular.z = no_vel_move
                        self.vel_pub.publish(cmd_vel)

                        # Remove the first element from the list to move to the next marker
                        self.marker_list.pop(0)
                        rospy.set_param('/marker_publisher/marker_list', self.marker_list)
                        self.allineato = False

                    elif error < pixel_thr and self.allineato == True:
                        # robot alligned with the center of the marker
                        print("robot and marker ALIGNED!")
                        cmd_vel = Twist()
                        cmd_vel.linear.x = lin_vel_move
                        cmd_vel.angular.z = no_vel_move
                        self.vel_pub.publish(cmd_vel)

                    else:
                        if self.allineato:
                            print("QQQQQQQQQQQQQQQQQQQQQQQQQQQ")
                            # CONTROLLER for robot's alignment with marker
                            cmd_vel = Twist()
                            cmd_vel.linear.x = self.kp_d * error
                            if cmd_vel.linear.x > self.ub_d:
                                cmd_vel.linear.x = self.ub_d
                           
                            if self.marker_center_x < width_camera:
                                cmd_vel.angular.z = self.kp_a * error
                                if cmd_vel.angular.z > self.ub_d:
                                    cmd_vel.angular.z = self.ub_d
                            else:
                                cmd_vel.angular.z = - self.kp_a * error
                                if cmd_vel.angular.z < - self.ub_d:
                                    cmd_vel.angular.z = - self.ub_d
                            
                            self.vel_pub.publish(cmd_vel)
                    

                else:
                   # if not self.allineato:
                    # allign the robot with camera
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    #if self.marker_center_x < width_camera:
                    cmd_vel.angular.z = self.kp_a * abs(yaw_error)#0.50
                    if cmd_vel.angular.z > self.ub_br:
                        cmd_vel.angular.z = self.ub_br
                            
                    self.vel_pub.publish(cmd_vel)
                            
                    vel_camera = Float64()
                    vel_camera.data =  - self.kp_a * abs(yaw_error) + 0.1 #-0.40
                    if vel_camera.data < - self.ub_cr:
                        vel_camera.data = - self.ub_cr
                    self.joint_state_pub.publish(vel_camera)
                    #else:
                     #   print("PORCODIO")

              
            else:
                
                vel_camera = Float64()
                print("CAMERA SEARCHING FOR MARKER...")
                vel_camera.data = ang_vel_move
                self.joint_state_pub.publish(vel_camera)
           
           
    
    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / math.fabs(angle)
        return angle
    
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

