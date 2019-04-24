#!/usr/bin/env python
import rospy
import rospkg
import serial
import tf
import numpy as np
from math import cos, sin, pi, exp, sqrt, atan2

from e190_bot.srv import *

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PointTracker:
    def __init__(self):

        # Initialize member fields
        self.current_robot_pose = Pose()
        #set the values of the pose
        self.current_target = Pose()
        self.current_path = Path()
        self.current_path_index = 0
        self.alpha = 0
        self.beta = 0
        self.rho = 0
        self.k_alpha = -6/8
        self.k_beta = 2/8
        self.k_rho = 0.5
        self.odom = Odometry()

        rospy.init_node('pointTracking', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        #Now get targets from the goal_path topic
        #self.target_sub = rospy.Subscriber('targetPose', Pose, self.target_callback)
        #self.path_sub = rospy.Subscriber('goal_Path', Path, self.path_callback)
        
        
        self.rate = rospy.Rate(20)    #10 Hz

        
        while not rospy.is_shutdown():
            self.cmd_vel_pub()
            self.rate.sleep()
        
    def target_callback(self, target):
        self.current_target.position = target.position
        #print("hi")

    def path_Service_client(self):
        rospy.wait_for_service('path_Service')
        try:
            path_Service_inst = rospy.ServiceProxy('path_Service', path_Service)
            current = PoseStamped()
            current.pose = self.odom.pose.pose
            resp1 = path_Service_inst(current)
            self.current_target = resp1.next.pose
            print("REceived target" + str(self.current_target))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def path_callback(self, path):
        self.current_path = path
        self.current_path_index = 0
        print("Received path: " + str(self.current_path.poses))
        self.current_target = self.current_path.poses[self.current_path_index].pose
    
    def odom_callback(self, odom):
        self.odom = odom
        delta_x = self.current_target.position.x - odom.pose.pose.position.x 
        delta_y = self.current_target.position.y - odom.pose.pose.position.y 
        self.rho = sqrt(delta_x ** 2 + delta_y ** 2)
        poseQuat = odom.pose.pose.orientation
        poseQuat_arr = np.array([poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w])
        eulerAngles = euler_from_quaternion(poseQuat_arr, 'sxyz')
        theta = eulerAngles[2]
        self.alpha = -theta + atan2(delta_y, delta_x)
        self.beta = theta + self.alpha
        # print("alpha:"+str(self.alpha) + "beta" + str(self.beta) + "rho:" + str(self.rho)+ "theta" + str(theta))

    def cmd_vel_pub(self):
        #update values
        delta_x = self.current_target.position.x - self.odom.pose.pose.position.x 
        delta_y = self.current_target.position.y - self.odom.pose.pose.position.y 
        self.rho = sqrt(delta_x ** 2 + delta_y ** 2)
        poseQuat = self.odom.pose.pose.orientation
        poseQuat_arr = np.array([poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w])
        eulerAngles = euler_from_quaternion(poseQuat_arr, 'sxyz')
        theta = eulerAngles[2]
        self.alpha = -theta + atan2(delta_y, delta_x)
        self.beta = theta + self.alpha
        # Compute desired velocities based on control laws
        v = self.k_rho * self.rho
        w = self.k_alpha * self.alpha + self.k_beta * self.beta
        if (self.rho < 0): # invalid rho, do nothing and wait for next callback, see elif case
            v = 0
            w = 0
        elif(self.rho < 0.05): #Once close enough to point, stop moving
            v = 0
            w = 0
            print("We've reached " + str(self.current_target.position))
            #update the index of the path to switch to the next target
            #if the last target, remains on that index
            #self.current_path_index = min(len(self.current_path.poses) -1, self.current_path_index+1)
            #if self.current_path.poses:
                #self.rho = -1 # Invalidate current rho so we know not to use it until next odom_callback updates it
                #self.current_target = self.current_path.poses[self.current_path_index].pose
                #print("Now targeting pose (" + str(self.current_path_index) +") " + str(self.current_target.position))
            self.path_Service_client()
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