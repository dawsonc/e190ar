#!/usr/bin/env python
import rospy
import rospkg
from xbee import XBee
import serial
import tf
import numpy as np
from math import cos, sin, pi, exp

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from e190_bot.msg import ir_sensor
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospack = rospkg.RosPack()

class botControl:
    ######################
    # constructor
    # creates an instance of the botControl object
    ######################
    def __init__(self):
        # create vars for hardware vs simulation
        self.robot_mode = "HARDWARE_MODE"#"SIMULATION_MODE"
        #self.control_mode = "MANUAL_CONTROL_MODE"

        # setup xbee communication, change ttyUSB0 to the USB port dongle is in
        if (self.robot_mode == "HARDWARE_MODE"):
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")
        
        print("Xbee setup successful")
        self.address = '\x00\x0C'#you may use this to communicate with multiple bots

        #init an odometry instance, and configure odometry info
        self.odom_init()

        #init a distance sensor instance
        self.ir_init()
        
        #init log file, "False" indicate no log will be made, log will be in e190_bot/data folder
        self.log_init(data_logging=False,file_name="log.txt")

        rospy.init_node('botControl', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        self.pubOdom = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.pubDistL = rospy.Publisher('/distL', ir_sensor, queue_size=10)
        self.pubDistC = rospy.Publisher('/distC', ir_sensor, queue_size=10)
        self.pubDistR = rospy.Publisher('/distR', ir_sensor, queue_size=10)
        self.time = rospy.Time.now()
        self.count = 0;
        self.r = 3.5*10**(-2)    #wheel radius is 3.5 cm
        self.L = 7.125*10**(-2)   #Wheel base radius 7.125 cm
        self.carpetCorrection = 1.3 #Correction factor for motor signal when on carpet

        self.rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.odom_pub();
            #print('hello')
            self.rate.sleep();

    ############################
    # ir_init
    # Initializes ir sensor by creating custom ir_sensor objects with a float32 distance datamember
    # Left, Center, Right
    ############################
    def ir_init(self):
         self.ir_L = ir_sensor() 
         self.ir_C = ir_sensor()
         self.ir_R = ir_sensor()

    ############################
    # odom_init
    # Initialize odometry using tf package broadcaster system
    ############################
    def odom_init(self):
        self.Odom = Odometry()
        self.Odom.header.frame_id = "odom_wheel"
        self.Odom.child_frame_id = "base_link"
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.encoder_resolution = 1.0/1440.0
        self.wheel_radius = .035 #unit in m
        self.L = 7.125*10**-2 # m
        self.theta = 0
        self.last_encoder_measurementL = 0
        self.last_encoder_measurementR = 0

    ############################
    # log_init
    # Initialize logging  
    ############################
    def log_init(self,data_logging=False,file_name="log.txt"):
        self.data_logging=data_logging
        if(data_logging):
            self.file_name = file_name
            self.make_headers();

    ############################
    # cmd_vel_callback
    # Receive a CmdVel object from human input via the xBee
    # CmdVel objects have a linear component and an angular component in m/s or rad/s
    # Translatetion along the x,y or z axis (x is forward, y to robot left, z vertical)
    # Rotation along the x,y, or z axis (ccw is positive, cw is negative)
    # Rotating right wheel forward produces ccw, left forward is cw
    # creates a motor driver command for the arduino code to take in
    ###########################
    def cmd_vel_callback(self,CmdVel):
        if(self.robot_mode == "HARDWARE_MODE"): 

            RCMD = ((CmdVel.linear.x)+ (CmdVel.angular.z)*self.L)/self.r * self.carpetCorrection
            LCMD = ((CmdVel.linear.x)- (CmdVel.angular.z)*self.L)/self.r * self.carpetCorrection

            LDIR = 0 if (LCMD >= 0) else 1      #formatting due to weird inheritance typing in python 
            RDIR = 0 if (RCMD >= 0) else 1

            RPWM =abs(int(RCMD/31.0838*255))          #pwm needs integer values from 1-255, not negative
            LPWM =abs(int(LCMD/30.5738*255))

            if (RPWM > 0):
                RPWM = max(RPWM, 25)
            if (LPWM > 0):
                LPWM = max(LPWM, 25)

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            # print("cmd: " + str(command))

            #command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.xbee.tx(dest_addr = self.address, data = command)

    ###################################
    # odom_pub
    # utilizes tf poses and transforms to format odometry data
    # publishes and logs the data
    ###################################
    def odom_pub(self):
        if(self.robot_mode == "HARDWARE_MODE"):
            self.count = self.count + 1
            print(self.count)
            command = '$S @'
            self.xbee.tx(dest_addr = self.address, data = command)
            try:
                update = self.xbee.wait_read_frame()
            except:
                pass

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:] #encoder readings are here, 2d array

            #print ("update sensors measurements ",encoder_measurements)
    

            #how about velocity?
            time_diff = rospy.Time.now() - self.time #look at valus from previous cycle

            #calculate difference in encoder position
            self.diffEncoderL = encoder_measurements[0] - self.last_encoder_measurementL
            self.diffEncoderR = encoder_measurements[1] - self.last_encoder_measurementR
            #save new encoder measurement for use next loop
            self.last_encoder_measurementL = encoder_measurements[0]
            self.last_encoder_measurementR = encoder_measurements[1]
            #Calculate the distance traveled by each wheel
            deltaSR = 2*pi*self.wheel_radius*self.diffEncoderR*self.encoder_resolution 
            deltaSL = 2*pi*self.wheel_radius*self.diffEncoderL*self.encoder_resolution

            # Compute local coordinate updates
            deltaS = (deltaSR + deltaSL) / 2.0
            deltaTheta = (deltaSR - deltaSL) / (2.0 * self.L)

            # Grab old global coordinate values
            px = self.Odom.pose.pose.position.x
            py = self.Odom.pose.pose.position.y
            # poseQuat = self.Odom.pose.pose.orientation
            # poseQuat_arr = np.array([poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w])
            # eulerAngles = euler_from_quaternion(poseQuat_arr, 'sxyz')
            # theta = eulerAngles[2]

            #update global coordinates
            self.Odom.pose.pose.position.x = px + deltaS * cos(self.theta+deltaTheta/2.0)
            self.Odom.pose.pose.position.y = py + deltaS * sin(self.theta+deltaTheta/2.0)
            self.Odom.pose.pose.position.z = .0
            self.theta = self.theta + deltaTheta #increment theta
            quat = quaternion_from_euler(.0, .0, self.theta)
            self.Odom.pose.pose.orientation.x = quat[0]
            self.Odom.pose.pose.orientation.y = quat[1]
            self.Odom.pose.pose.orientation.z = quat[2]
            self.Odom.pose.pose.orientation.w = quat[3]

            # #https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            self.odom_broadcaster.sendTransform(
                (self.Odom.pose.pose.position.x, self.Odom.pose.pose.position.y, .0),
                tf.transformations.quaternion_from_euler(.0, .0, 1.57),
                rospy.Time.now(),
                self.Odom.child_frame_id,
                self.Odom.header.frame_id,
            )

            self.pubOdom.publish(self.Odom) #we publish in /odom topic

            #about range sensors, update here
            range_measurements = data[:-2] #range readings are here, 3d array F, L, R
            # self.pubRangeSensor(range_measurements)
            # print ("update ir measurements ",range_measurements)

        if(self.data_logging):
            self.log_data();

        self.time = rospy.Time.now()

    ############################
    # ir_cal
    # Calibration Curve for ir distance sensors
    #rdg = A*d^B power fit
    ############################
    def ir_cal(self,range):
        A = 13895.0  #Fit coefficients for power fit
        B = -0.929
        dist = (range/A) ** (1/B)
        return dist

    ############################
    # pubRangeSensor
    # pubish ir sensor data after converted to actual distances
    ############################
    def pubRangeSensor(self,ranges):
         self.ir_L.distance = self.ir_cal(ranges[1]) #range was in F,L,R order so need to rearange
         self.ir_C.distance = self.ir_cal(ranges[0])
         self.ir_R.distance = self.ir_cal(ranges[2])

         print("Distance Values", self.ir_L.distance)
        
         self.pubDistL.publish(self.ir_L)
         self.pubDistC.publish(self.ir_C)
         self.pubDistR.publish(self.ir_R)

    ############################
    # make_headers
    # Format logging file
    ############################
    def make_headers(self):
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.close()

    ############################
    # log_data
    # logs data when odometry is being published
    # Not being done yet
    ############################
    def log_data(self):
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')

        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [1,2,3,self.Odom.pose.pose.position.x,self.Odom.pose.pose.position.y]]
        
        f.write(' '.join(data) + '\n')#maybe you don't want to log raw data??
        f.close()

#Will initialize a new botControl object
if __name__ == '__main__':
    try:
        bot = botControl()

    except rospy.ROSInterruptException:
        pass