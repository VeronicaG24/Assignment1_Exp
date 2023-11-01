#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

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

VERBOSE = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        # topic where we publish
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Aggiunto il publisher per il messaggio JointState
        self.joint_state_pub = rospy.Publisher("/robot_exp/camera_position_controller/command", Float64, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed",
                                           CompressedImage, self.callback, queue_size=1)
        # subscribed Topic
        self.subscriber_ack = rospy.Subscriber("/ack_camera",
                                           Bool, self.ack_callback, queue_size=1)
        self.ack_data=False

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''

        #print('received image of type: "%s"' % ros_data.format)
        msgs = Float64()
        msgs.data=1.0
        self.joint_state_pub.publish(msgs)

        print('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ##
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        msgs = Float64()
        if not(self.ack_data):
          msgs.data=0.5
          
          self.joint_state_pub.publish(msgs)
          
        else :
          print("lllll")

        # Update the points queue
        # pts.appendleft(center)
        #cv2.imshow('window', image_np)
        #cv2.waitKey(2)
        

        #self.subscriber.unregister()
        
    def ack_callback(self, ack):
        #rospy.loginfo("Ack received: %s", ack.data)
        msgs = Float64()
        if ack.data:
           msgs.data=0.0
           self.joint_state_pub.publish(msgs)
           self.ack_data=ack.data
           print("Ack is: "+str(self.ack_data))
           
        

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
