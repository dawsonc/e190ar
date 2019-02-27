#!/usr/bin/env python
# Written by C. Dawson based on Prof. Chang's simple_tf_listener.py code
import rospy
import rospkg
import numpy as np

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

rospack = rospkg.RosPack()

class goal_pose_transformer:

    def __init__(self):
        rospy.init_node('goal_pose_transformer', anonymous=True)
        self.goal_pose_init()

        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.tf_lis();
            self.rate.sleep();

    def goal_pose_init(self):

        self.goalPose=PoseStamped()# goal pose, in odom_wheel frame
        self.goalPoseLocal = PoseStamped()# goal pose, in local base_link frame
        
        self.goalPose.header.frame_id = "odom_wheel"
        self.goalPoseLocal.header.frame_id = "base_link"

        # We are given goal pose at position (2, 2, 0) in the Self-Study 1 wiki
        self.goalPose.pose.position.x = 2.0
        self.goalPose.pose.position.y = 2.0
        self.goalPose.pose.position.z = .0
        
        # We are given goal pose at angle (0, 0, 1, 0) in the Self-Study 1 wiki
        self.goalPose.pose.orientation.x = 0.0
        self.goalPose.pose.orientation.y = 0.0
        self.goalPose.pose.orientation.z = 1
        self.goalPose.pose.orientation.w = 0.0


    def tf_lis(self):
        # We want the transform from odom_wheel to base_link, so we use the order below
        self.listener.waitForTransform("base_link", "odom_wheel", rospy.Time(0),rospy.Duration(2.0))
        (trans,rot) = self.listener.lookupTransform("base_link", "odom_wheel", rospy.Time(0))
        
        self.goalPoseLocal = self.listener.transformPose("base_link",self.goalPose)
        print(self.goalPoseLocal)


if __name__ == '__main__':
    try:
        tf = goal_pose_transformer()

    except (rospy.ROSInterruptException):
        pass