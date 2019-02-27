#!/usr/bin/env python
import rospy
import rospkg

from geometry_msgs.msg import Twist

def cmd_vel_Publisher():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_Publisher', anonymous=True)
    rate = rospy.Rate(10)    #10 Hz
    startTime = rospy.get_time()
    state = 0 # 0 for forward, 1 for turn, 2 for stop

    
    while not rospy.is_shutdown():
        twist = Twist()        #message object is a Twist
        deltaT = rospy.get_time() - startTime
        #Go through the steps of a square.  Used calibration for carpet
        if deltaT < 4:
            state = 0
        elif (4 <= deltaT and deltaT < 5):
            state = 1
        elif (5 <= deltaT and deltaT < 9):
            state = 0
        elif (9 <= deltaT and deltaT < 10):
            state = 1
        elif (10 <= deltaT and deltaT < 14):
            state = 0
        elif (14 <= deltaT and deltaT < 15):
            state = 1
        elif (15 <= deltaT and deltaT < 19):
            state = 0
        else:
            state = 2

        if state == 0:
            twist.linear.x = 0.25  #go straight state

        elif state == 1:
            twist.angular.z = 3.7/2 #Empirical value to turn 90 degrees state
        else:
            pass

        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()  

if __name__ == '__main__':
    try:
        cmd_vel_Publisher()
    except rospy.ROSInterruptException:
        pass