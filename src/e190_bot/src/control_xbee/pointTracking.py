#!/usr/bin/env python
import rospy
import rospkg
import serial
import tf
import numpy as np
from math import cos, sin, pi, exp, sqrt, atan2

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PointTracker:
    def __init__(self):

        # Initialize member fields
        self.current_robot_pose = Pose()
        #set the values of the pose
        self.current_target = Pose()
        self.alpha = 0
        self.beta = 0
        self.rho = 0
        self.k_alpha = -3/8
        self.k_beta = 5/8
        self.k_rho = 1

        rospy.init_node('pointTracking', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.target_sub = rospy.Subscriber('targetPose', Pose, self.target_callback)
        self.rate = rospy.Rate(5)    #10 Hz

        
        while not rospy.is_shutdown():
            self.cmd_vel_pub()
            self.rate.sleep()
        
    def target_callback(self, target):
        self.current_target.position = target.position
        print("hi")
    
    def odom_callback(self, odom):
        delta_x = self.current_target.position.x - odom.pose.pose.position.x 
        delta_y = self.current_target.position.y - odom.pose.pose.position.y 
        self.rho = sqrt(delta_x ** 2 + delta_y ** 2)
        poseQuat = odom.pose.pose.orientation
        poseQuat_arr = np.array([poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w])
        eulerAngles = euler_from_quaternion(poseQuat_arr, 'sxyz')
        theta = eulerAngles[2]
        self.alpha = -theta + atan2(delta_y, delta_x)
        self.beta = theta + self.alpha
        print("alpha:"+str(self.alpha) + "beta" + str(self.beta) + "rho:" + str(self.rho)+ "theta" + str(theta))

    def cmd_vel_pub(self):
        # Compute desired velocities based on control laws
        v = self.k_rho * self.rho
        w = self.k_alpha * self.alpha + self.k_beta * self.beta
        if(self.rho < 0.05): #ONce close enough to point, stop moving
            v = 0
            w = 0
        
        # Create message and publish
        twist = Twist()        #message object is a Twist
        twist.linear.x = v
        twist.angular.z = w
        #rospy.loginfo(twist)
        self.pub.publish(twist)


if __name__ == '__main__':
    try:
        tracker = PointTracker()
    except rospy.ROSInterruptException:
        pass