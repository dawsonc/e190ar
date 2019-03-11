#!/usr/bin/env python
import rospy
import rospkg
import serial
import tf
import numpy as np
from math import cos, sin, pi, exp, sqrt, atan2

from geometry_msgs.msg import Transform, Twist
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class marioKart_Controller:
    def __init__(self):
        
            self.eulerAngles =  [pi, 0, 0 ]
            rospy.init_node('echo_euler_angles', anonymous=True)
            self.transform_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transform_callback)
            self.pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)

            
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
            print(self.eulerAngles)
        
        print("hi")

    def cmd_vel_pub(self):
        # Create message and publish based on euler angles
        twist = Twist()        #message object is a Twist
        if self.eulerAngles[0] > 0 :
            twist.linear.x = pi- self.eulerAngles[0] 
        if self.eulerAngles[0] < 0 :
            twist.linear.x = -pi - self.eulerAngles[0]
        if abs(twist.linear.x) < 0.05*pi:
            twist.linear.x =0
        #need to set a scaling, maybe subtract a value
        twist.angular.z = self.eulerAngles[2]
        #rospy.loginfo(twist)
        self.pub.publish(twist)

if __name__ == '__main__':
    try: 
        marioKart = marioKart_Controller()
    except rospy.ROSInterruptException:
        pass