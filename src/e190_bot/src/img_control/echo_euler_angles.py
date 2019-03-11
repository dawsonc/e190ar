#!/usr/bin/env python
import rospy
import rospkg
import serial
import tf
import numpy as np
from math import cos, sin, pi, exp, sqrt, atan2

from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def transform_callback(transformsArray):
    if transformsArray.transforms:
        #eulerAngles = euler_from_quaternion(transformsArray.transforms[0].transform.rotation, 'sxyz')
        poseQuat = transformsArray.transforms[0].transform.rotation
        poseQuat_arr = np.array([poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w])
        eulerAngles = euler_from_quaternion(poseQuat_arr, 'sxyz')
        print(eulerAngles)
    
    print("hi")

if __name__ == '__main__':
    try:
        rospy.init_node('echo_euler_angles', anonymous=True)
        transform_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, transform_callback)
        rate = rospy.Rate(5)    #10 Hz

        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass