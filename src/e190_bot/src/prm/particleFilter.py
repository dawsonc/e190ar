#!/usr/bin/env python
import rospy
import rospkg

import tf

import random
import math
import numpy as np
from scipy.stats import norm as scinorm

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from e190_bot.msg import ir_sensor
from particle import Particle


from tf.transformations import euler_from_quaternion, quaternion_from_euler

class particleFilter:

    def __init__(self):

        rospy.init_node('particleFilter', anonymous=True)

        self.particles = [] #an array of all particles
        self.N = 5 #number of particles
        
        self.map_init()
        self.particle_init()

        self.encoder_resolution = 1.0/1440.0
        self.wheel_radius = .035 #unit in m
        self.L = 7.125*10**-2 # m

        self.odomest_broadcaster = tf.TransformBroadcaster()
        self.OdomEst = Odometry()
        self.OdomEst.header.frame_id = "odom_wheel"
        self.OdomEst.child_frame_id = "base_link"

        self.particle_broadcasters = []
        for i in range(0, self.N):
            self.particle_broadcasters.append(tf.TransformBroadcaster())

        self.distL = 0 
        self.distC = 0 
        self.distR = 0

        self.prev_distL = 0 
        self.prev_distC = 0 
        self.prev_distR = 0

        rospy.Subscriber("/distL", ir_sensor, self.irL_callback)
        rospy.Subscriber("/distC", ir_sensor, self.irC_callback)
        rospy.Subscriber("/distR", ir_sensor, self.irR_callback)
        rospy.Subscriber("/encoder", Vector3, self.encoder_callback)

        self.pubOdom = rospy.Publisher("/odom_est", Odometry, queue_size=10)
        self.rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def particle_init(self):
        for n in range(0,self.N):
            #populate array with uniformly random particles-, but close to the origin
            #populate around first eighth
            x = random.uniform(0,self.map_width/8 * self.map_res*0.99)
            y = random.uniform(0, self.map_height/8 * self.map_res*0.99)
            # theta = random.uniform(0,2*np.pi)
            theta = random.uniform(-0.2,0.2)
            newParticle = Particle(x,y,theta)
            # print("new particle at (%f, %f)" % (x, y))
            self.particles.append(newParticle) 

    def map_init(self):
        rospy.wait_for_service('static_map')
        try:
            map_Service = rospy.ServiceProxy('static_map', GetMap)
            self.map = map_Service().map
            self.map_width = self.map.info.width
            self.map_height = self.map.info.height
            self.map_res = self.map.info.resolution

            # print(self.map.data)
            # print("Map size: " + str(self.map_res*self.map_width) +" , " + str(self.map_res*self.map_height))
        except rospy.ServiceException, e:
            print "Map service call failed: %s"%e


    def encoder_callback(self, encoderVector):
        # Propagatte particles

        #receive encoder measuremtns from publisher
        self.diffEncoderL = encoderVector.x
        self.diffEncoderR = encoderVector.y

        belief = self.localize_estimate_with_particles(
            self.diffEncoderL, self.diffEncoderR,
            self.distL, self.distC, self.distR
        )

        self.OdomEst.pose.pose.position.x = belief.x
        self.OdomEst.pose.pose.position.y = belief.y
        self.OdomEst.pose.pose.position.z = .0
        quat = quaternion_from_euler(.0, .0, belief.theta)
        self.OdomEst.pose.pose.orientation.x = quat[0]
        self.OdomEst.pose.pose.orientation.y = quat[1]
        self.OdomEst.pose.pose.orientation.z = quat[2]
        self.OdomEst.pose.pose.orientation.w = quat[3]

        self.pubOdom.publish(self.OdomEst) #we publish in /odom topic

        self.odomest_broadcaster.sendTransform(
            (self.OdomEst.pose.pose.position.x, self.OdomEst.pose.pose.position.y, .0),
            quat,
            rospy.Time.now(),
            self.OdomEst.child_frame_id,
            self.OdomEst.header.frame_id,
        )

        for i in range(0, self.N):
            p = self.particles[i]
            pb = self.particle_broadcasters[i]
            quat = quaternion_from_euler(.0, .0, p.theta)
            pb.sendTransform(
                (p.x, p.y, .0),
                quat,
                rospy.Time.now(),
                "particle"+str(i),
                self.OdomEst.header.frame_id,
            )


    def irL_callback(self, distL):
        self.distL = distL.distance/100
        #print("L dist: %f" % self.distL)

    def irC_callback(self, distC):
        self.distC = distC.distance/100
        #print("C dist: %f" % self.distC)

    def irR_callback(self, distR):
        self.distR = distR.distance/100
        #print("R dist: %f" % self.distR)

    def localize_estimate_with_particles(self, encL, encR, distL, distC, distR):
        # for each particle, first update using encoder measurements,
        # then update particle weights based on distance measurements 
        print("==============Localizing===========")
        max_weight = 0 # we'll need this later
        for i in range(0, self.N):
            self.propagate(self.particles[i], encL, encR)
            w = self.updateWeight(self.particles[i], distL, distC, distR)
            if w > max_weight:
                max_weight = w
        meas_dists = [distL, distC, distR]
        prev_dists = [self.prev_distL, self.prev_distC, self.prev_distR]

        #if measured distances are different enough from the previous then resample
        if sum([np.abs(distL - self.prev_distL), np.abs(distC-self.prev_distC), np.abs(distR-self.prev_distR)]) > 0.5:
            print("Resampling" + str(sum([np.abs(distL - self.prev_distL), np.abs(distC-self.prev_distC), np.abs(distR-self.prev_distR)])))
            
            # now resample the particles based on their weights,
            # using Prof. Clark's approximate sampling method
            temp_particles = []
            for i in range(0, self.N):
                w = self.particles[i].weight
                w = w / max_weight
                if w < 0.25:
                    temp_particles.append(self.particles[i])
                elif w < 0.5:
                    temp_particles.append(self.particles[i])
                    temp_particles.append(self.particles[i])
                elif w < 0.75:
                    temp_particles.append(self.particles[i])
                    temp_particles.append(self.particles[i])
                    temp_particles.append(self.particles[i])
                elif w <= 1:
                    temp_particles.append(self.particles[i])
                    temp_particles.append(self.particles[i])
                    temp_particles.append(self.particles[i])
                    temp_particles.append(self.particles[i])
            
            # now draw uniformly from temp_particles, calculating total weight as we go

            for i in range(0, self.N):
                r = int(random.uniform(0, 1)*len(temp_particles))
                self.particles[i] = temp_particles[r]
          

            #udate prvious measuremtns
            self.prev_distC = distL
            self.prev_distL = distC
            self.prev_distR = distR

        # get location estimate based on weighted average of particle locations
        belief = Particle()
        total_w = 0
        for i in range(0, self.N):
            belief.x += self.particles[i].x * self.particles[i].weight
            belief.y += self.particles[i].y * self.particles[i].weight
            belief.theta += self.particles[i].theta * self.particles[i].weight
            total_w += self.particles[i].weight
        
        belief.x *= 1/total_w
        belief.y *= 1/total_w
        belief.theta *= 1/total_w

        print("I think the particle is at (%f, %f, %f)" % (belief.x, belief.y, belief.theta))



        return belief


    def propagate(self, particle, encL, encR):
        print("propagating particle at (%f, %f)" % (particle.x, particle.y))
        #Calculate the distance traveled by each wheel
        deltaSR = 2*np.pi*self.wheel_radius*self.diffEncoderR*self.encoder_resolution 
        deltaSL = 2*np.pi*self.wheel_radius*self.diffEncoderL*self.encoder_resolution

        # Add some random noise to the wheel distances
        wheelNoise = 0.001 # noise on wheel distance
        # deltaSL = deltaSL+random.uniform(-wheelNoise, wheelNoise)
        # deltaSR = deltaSR+random.uniform(-wheelNoise, wheelNoise)

        # Compute local coordinate updates
        deltaS = (deltaSR + deltaSL) / 2.0
        deltaTheta = (deltaSR - deltaSL) / (2.0 * self.L)
        

        # print(deltaS)
        # print(deltaTheta)

        # update particle
        particle.x += deltaS*np.cos(particle.theta) + random.gauss(0, wheelNoise)
        particle.y += deltaS*np.sin(particle.theta) + random.gauss(0, wheelNoise)
        particle.theta += deltaTheta

        # restrict particles to within map box
        particle.x = min(particle.x, self.map_width * self.map_res)
        particle.y = min(particle.y, self.map_height * self.map_res)
        particle.x = max(particle.x, 0)
        particle.y = max(particle.y, 0)

        #restrict particles to within path space
        particle
        


    def findDistances(self, particle):
        #max range is 150 cm=0.15 m
        #collision detect iteratively in the three directions up to 150 cm
        
        collisionL = self.collisionDetect(particle.x, particle.y, 
            particle.x+np.cos(particle.theta+np.pi/2)*1.50,particle.y+np.sin(particle.theta+np.pi/2)*1.50 )
        collisionC = self.collisionDetect(particle.x, particle.y, 
            particle.x+np.cos(particle.theta)*1.50,particle.y+np.sin(particle.theta)*1.50 )
        collisionR = self.collisionDetect(particle.x, particle.y, 
            particle.x+np.cos(particle.theta-np.pi/2)*1.50,particle.y-np.sin(particle.theta+np.pi/2)*1.50 )
        
        if collisionL is not None:
            collL_pos = self.grid_to_pos(collisionL[0],collisionL[1] )
            distL = self.distance(particle.x, particle.y, collL_pos[0], collL_pos[1])
        else:
            distL = -1 # outside of max range, so invalidate
        if distL < 0.2:
            distL = -1 # inside min range, so invalidate

        if collisionC is not None:
            collC_pos = self.grid_to_pos(collisionC[0],collisionC[1] )
            distC = self.distance(particle.x, particle.y, collC_pos[0], collC_pos[1])
        else:
            distC = -1 # outside of max range, so invalidate
        if distL < 0.2:
            distC = -1 # inside min range, so invalidate

        if collisionR is not None:
            collR_pos = self.grid_to_pos(collisionR[0],collisionR[1] )
            distR = self.distance(particle.x, particle.y, collR_pos[0], collR_pos[1])
        else:
            distR = -1 # outside of max range, so invalidate
        if distL < 0.2:
            distR = -1 # inside min range, so invalidate
        
        return distL, distC, distR

    def updateWeight(self, particle, distL, distC, distR):
        # update weight of particle based on difference
        # between expected and measured distances

        # print("updating weight of particle at (%f, %f)" % (particle.x, particle.y))

        meas_dists = [distL, distC, distR]

        # calculate expected distances for each sensor
        map_distL, map_distC, map_distR = self.findDistances(particle)
        map_dists = [map_distL, map_distC, map_distR]

        weight = 0.01 # accumulate weight by averaging all three probabilities

        # construct gaussian for each sensor measurement
        measurement_stddev = 14 # out of 1024
        range_stddev = 0.1
        A = 13895.0  #Fit coefficients for sensor model power fit
        B = -0.929 # model is reading = A * (range**B)
        for i in range(0, 3):
            # if distance is -1, no collision, so we ignore that information
            if map_dists[i] == -1:
                continue
            # linearize sensor model to convert to range
            slope = A * B * (map_dists[i])**(B-1)
            # range_stddev = measurement_stddev * 1/slope

            # draw weight from normal distribution
            # print("map dist: %f, meas_dist: %f" % (map_dists[i], meas_dists[i]))
            weight += scinorm(map_dists[i], range_stddev).pdf(meas_dists[i]) # cite: russel bingham

        particle.weight = weight / 3
        # print(weight/3)

        return weight

    def distance(self, x1, y1, x2, y2):
        dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return dist

    def grid_to_pos(self, gridX, gridY):
        posx = gridX * self.map_res + self.map.info.origin.position.x
        posy = gridY * self.map_res + self.map.info.origin.position.y

        return posx, posy

    #convert position in meter to map grid id, return grid_x, grid_y and their 1d grid_id
    def pos_to_grid(self,poseX,poseY):
        grid_i = int((poseX - self.map.info.origin.position.x) / self.map_res)
        grid_j = int((poseY - self.map.info.origin.position.y) / self.map_res)

        grid_id = grid_j * self.map_width + grid_i

        return grid_i, grid_j, grid_id

    #straight line collision detection, all inputs unit are in meter
    def collisionDetect(self,x1,y1,x2,y2):
        grid_i1, grid_j1, grid_id1 = self.pos_to_grid(x1, y1)
        grid_i2, grid_j2, grid_id2 = self.pos_to_grid(x2, y2)

        # print ("Check line between: (%f, %f) and (%f, %f)" % (x1, y1, x2, y2))
        # print ("Check line between: (%d, %d) and (%d, %d)" % (grid_i1, grid_j1, grid_i2, grid_j2))
        line = get_line(grid_i1, grid_j1, grid_i2, grid_j2)
        #print(line)

        for k in range(0,len(line)):
            # print("Check map gird: " + str(line[k][0]) + " " + str(line[k][1]))
            # print(line[k][1] * self.map_width + line[k][0])
            
            #Map value 0 - 100
            if(self.map.data[line[k][1] * self.map_width + line[k][0]]!=0):
                return line[k] #returns x,y in gridspace

        return None

    


#Bresenham Line alogirthm from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python
def get_line(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points  



if __name__ == '__main__':
    try:
        filter = particleFilter()
        

    except rospy.ROSInterruptException:
        pass