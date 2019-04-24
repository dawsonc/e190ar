#!/usr/bin/env python
import rospy
import rospkg
import serial
import tf
import numpy as np
from math import cos, sin, pi, exp, sqrt, atan2

from geometry_msgs.msg import Transform, Twist, Vector3
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class marioKart_Controller:
    def __init__(self):
        
            self.eulerAngles =  [pi, 0, 0 ]
            rospy.init_node('echo_euler_angles', anonymous=True)
            self.transform_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transform_callback)
            self.pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)

            self.poseTranslation = Vector3()
            self.poseTranslation.z = 0.5
            
            rate = rospy.Rate(5)    #10 Hz

            while not rospy.is_shutdown():
                self.cmd_vel_pub()
                rate.sleep()

    def transform_callback(self, transformsArray):
        if transformsArray.transforms:
            #eulerAngles = euler_from_quaternion(transformsArray.transforms[0].transform.rotation, 'sxyz')
            poseQuat = transformsArray.transforms[0].transform.rotation
            poseQuat_arr = np.array([poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w])
            self.eulerAngles = euler_from_quaternion(poseQuat_arr, 'sxyz')
            # also get translation
            self.poseTranslation = transformsArray.transforms[0].transform.translation
            # print(self.eulerAngles)
        
        # print("hi")

    def cmd_vel_pub(self):
        # Create message and publish based on euler angles
        twist = Twist()        #message object is a Twist
        
        # this code sets speed based on distance to aruco
        print(self.poseTranslation.z)
        twist.linear.x = self.poseTranslation.z - 0.5
        if abs(twist.linear.x) < 0.05:
            twist.linear.x = 0
        twist.linear.x *= 2

        ## this code sets speed based on tilting forward and backward
        # if self.eulerAngles[0] > 0 :
        #     twist.linear.x = pi- self.eulerAngles[0] 
        # if self.eulerAngles[0] < 0 :
        #     twist.linear.x = -pi - self.eulerAngles[0]
        # if abs(twist.linear.x) < 0.05*pi:
        #     twist.linear.x =0
        #need to set a scaling, maybe subtract a value
        
        twist.angular.z = 1.5*self.eulerAngles[2] # no sign change for laptop control, minus sign for wheel control
        print(self.eulerAngles[2])
        if abs(twist.angular.z) < 0.1:
            twist.angular.z = 0
        
        # twist.linear.x = - twist.linear.x # make sure controls act the intuitive way when you hold the wheel and use tilt control
        #rospy.loginfo(twist)
        self.pub.publish(twist)

if __name__ == '__main__':
    try: 
        marioKart = marioKart_Controller()
    except rospy.ROSInterruptException:
        pass