#!/usr/bin/env python
import rospy
import rospkg
import numpy as np

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospack = rospkg.RosPack()

class simple_tf:

    def __init__(self):

        self.px = .0
        self.py = .0
        self.theta = .0
        self.ds = .2 # hard code here, should come from odometry
        self.dtheta = .5 # hard code here, should come from odometry 

        rospy.init_node('simple_tf', anonymous=True)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.ir_L_id = "ir_L"
        self.ir_R_id = "ir_R"
        self.ir_C_id = "ir_C"
        self.child_frame_id = "base_link"
        self.ir_L_broadcaster = tf.TransformBroadcaster()
        self.ir_R_broadcaster = tf.TransformBroadcaster()
        self.ir_C_broadcaster = tf.TransformBroadcaster()
        self.encoder_resolution = 1.0/1440.0
        self.wheel_radius = .035 #unit in m
        self.L = 7.125*10**-2 # m

        self.rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.tf_pub();
            self.rate.sleep();


    def tf_pub(self):
        # #https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        self.odom_broadcaster.sendTransform(
            (self.px, self.py, .0),
            tf.transformations.quaternion_from_euler(.0, .0, self.theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        #ir_L_broadcaster
        self.ir_L_broadcaster.sendTransform(
            (0, self.L/2, 2*self.wheel_radius),
            tf.transformations.quaternion_from_euler(pi/2, .0, .0, 'sxyz'),
            rospy.Time.now(),
            self.ir_L_id,
            self.child_frame_id,
        )

        #ir_R_broadcaster
        self.ir_R_broadcaster.sendTransform(
            (0, -self.L/2, 2*self.wheel_radius),
            tf.transformations.quaternion_from_euler(pi/2, .0, pi,'sxyz'),
            rospy.Time.now(),
            self.ir_R_id,
            self.child_frame_id,
        )

        #ir_C_broadcaster
        self.ir_C_broadcaster.sendTransform(
            (self.L/2,0, self.wheel_radius),
            tf.transformations.quaternion_from_euler(pi/2, .0, pi/2,'sxyz'),
            rospy.Time.now(),
            self.ir_C_id,
            self.child_frame_id,
        )

        self.px = self.ds * np.cos(self.theta+self.dtheta/2) + self.px
        self.py = self.ds * np.sin(self.theta+self.dtheta/2) + self.py
        self.theta += self.dtheta

if __name__ == '__main__':
    try:
        tf = simple_tf()

    except rospy.ROSInterruptException:
        pass
