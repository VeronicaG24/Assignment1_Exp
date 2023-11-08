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

VERBOSE = False

vel_camera = 0.5

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        # topic where we publish
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Aggiunto il publisher per il messaggio JointState
        self.joint_state_pub = rospy.Publisher("/robot_exp/camera_velocity_controller/command", Float64, queue_size=1)

        # subscribed Topic
        self.subscriber_joint_pos = rospy.Subscriber("/gazebo/link_states", LinkState, self.pos_callback, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.move_callback, queue_size=1)
        # subscribed Topic
        self.subscriber_ack = rospy.Subscriber("/ack_camera", Bool, self.ack_callback, queue_size=1)

        # Subscriber for the marker center
        self.marker_center_sub = rospy.Subscriber("/marker_point", Point, self.marker_center_callback, queue_size=1)

        self.pixel_side_sub = rospy.Subscriber("/pixel_side_marker", Float64, self.pixel_callback, queue_size=1)

        self.ack_data = False
        self.can_move = False
        self.current_pixel_side = 0.0

        self.stop = False

        self.position_camera_x = 0.0
        self.position_camera_y = 0.0
        self.position_camera_z = 0.0

        self.position_robot_x = 0.0
        self.position_robot_y = 0.0
        self.position_robot_z = 0.0

        self.center_x = 0.0
        self.center_y = 0.0

        # Get the parameter for operation mode -> 0: camera fixed, 1: camera moving
        self.mode = rospy.get_param('/marker_publisher/mode')

    def pos_callback(self, data):
        try:
            idx_camera = data.name.index("rosbot::camera_link")
            idx_base = data.name.index("rosbot::base_link")
        except ValueError:
            rospy.logerr("Link 'rosbot::camera_link' or 'rosbot::base_link' not found in LinkStates message")
            return

        self.position_camera_x = data.pose[idx_camera].position.x
        self.position_camera_y = data.pose[idx_camera].position.y
        self.position_camera_z = data.pose[idx_camera].position.z

        self.position_robot_x = data.pose[idx_base].position.x
        self.position_robot_y = data.pose[idx_base].position.y
        self.position_robot_z = data.pose[idx_base].position.z

        print("The camera position is: " + str(self.position_camera_z))
        print("The robot position is: " + str(self.position_robot_z))

    def move_callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        #### direct conversion to CV2 ##
        np_arr = np.frombuffer(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        if self.mode == 0:
            vel = Twist()
            if not self.ack_data:
                vel.linear.x = 0.0
                vel.angular.z = 1.0
                
            self.vel_pub.publish(vel)
            print("Ack is: " + str(self.ack_data))
            print("Vel is: " + str(vel.angular.z))

            if self.can_move:
                print("MI MUOVO!!")
                if self.current_pixel_side > 170:
                    print("\n********** CI SONO ARRIVATO! **********\n")
                    self.stop = True
                    # Se current_pixel_side è maggiore di 200, ferma il robot
                    vel = Twist()
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    self.vel_pub.publish(vel)

                else:
                    # Calcola le velocità lineari e angolari per muovere il robot verso center_x e center_y
                    # Puoi regolare questi valori in base alle tue esigenze

                    # sradius = math.atan2(self.center_y, self.center_x)

                    # Build twist msg
                    if not self.stop:
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.1  # lin_control
                        cmd_vel.angular.z = 0.0

                        self.vel_pub.publish(cmd_vel)
        
        elif self.mode == 1:
            msgs = Float64()
            if not self.ack_data:
                msgs.data = vel_camera

            self.joint_state_pub.publish(msgs)

            print("Ack is: " + str(self.ack_data))
            print("Msg is: " + str(msgs.data))

            if self.can_move:
                print("MI MUOVO!!")
                if self.current_pixel_side > 170:
                    print("\n********** CI SONO ARRIVATO! **********\n")
                    self.stop = True
                    # Se current_pixel_side è maggiore di 200, ferma il robot
                    vel = Twist()
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    self.vel_pub.publish(vel)

                else:
                    # Calcola le velocità lineari e angolari per muovere il robot verso center_x e center_y
                    # Puoi regolare questi valori in base alle tue esigenze

                    # sradius = math.atan2(self.center_y, self.center_x)

                    # Build twist msg
                    if not self.stop:
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.1  # lin_control
                        cmd_vel.angular.z = 0.0

    def ack_callback(self, ack):
        # rospy.loginfo("Ack received: %s", ack.data)
        msgs = Float64()
        self.ack_data = ack.data
        if ack.data:
            msgs.data = 0.0
            self.joint_state_pub.publish(msgs)
            # fai ruotare il robot per farlo allineare alla camera:
            # due ruote in un senso e due nell'altro e la camera a velocità opposta

        print("Ack is: " + str(self.ack_data))
        print("Msg is: " + str(msgs.data))

    def marker_center_callback(self, data):
        """if abs(self.position_robot_z - self.position_camera_z) <= 0.05:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.vel_pub.publish(vel)

            msgs = Float64()
            msgs.data = 0.0
            self.joint_state_pub.publish(msgs)

            self.can_move = True

        else:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.5
            self.vel_pub.publish(vel)

            msgs = Float64()
            msgs.data = -0.5
            self.joint_state_pub.publish(msgs)

        print("The camera position is: " + str(self.position_camera_z))
        print("The robot position is: " + str(self.position_robot_z))
	"""
        self.can_move = True
        center_x = data.x
        center_y = data.y
        rospy.loginfo("Center XXXXXXXX: %f, Center YYYYYY: %f", center_x, center_y)

    def pixel_callback(self, data):
        self.current_pixel_side = data.data
        rospy.loginfo("PIXELLLLL: %f", self.current_pixel_side)

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

