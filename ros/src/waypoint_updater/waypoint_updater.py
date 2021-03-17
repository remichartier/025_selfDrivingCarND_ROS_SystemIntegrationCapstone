#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Notes from project lesson : 
===========================

This package contains the waypoint updater node: waypoint_updater.py. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.

Suggested Order of Project Development
=======================================
Because you will be writing code across several packages with some nodes depending on messages published by other nodes, we suggest completing the project in the following order:

1. Waypoint Updater Node (Partial): Complete a partial waypoint updater which subscribes to /base_waypoints and /current_pose and publishes to /final_waypoints.

The /base_waypoints topic publishes a list of all waypoints for the track, so this list includes waypoints both before and after the vehicle (note that the publisher for /base_waypoints publishes only once). For this step in the project, the list published to /final_waypoints should include just a fixed number of waypoints currently ahead of the vehicle:

The first waypoint in the list published to /final_waypoints should be the first waypoint that is currently ahead of the car.
The total number of waypoints ahead of the vehicle that should be included in the /final_waypoints list is provided by the LOOKAHEAD_WPS variable in waypoint_updater.py.

We can see that both the /final_waypoints and /base_waypoints topics have message type Lane. You can look at the details about this message type in <path_to_project_repo>/ros/src/styx_msgs/msg/.

message type being used in /final_waypoints:

Type: styx_msgs/Lane

provides the following message information:

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
styx_msgs/Waypoint[] waypoints
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  geometry_msgs/TwistStamped twist
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z

From here you can see that the messages contain a header and a Waypoint list named waypoints. Each waypoint has pose and twist data. Going further, you can see that twist.twist data contains 3D linear and angular velocities.

Lane message example
=====================
As a use-case example, given a single styx_msgs/Lane message my_lane_msg, you can access the x direction linear velocity of the first waypoint in Python with:

my_lane_msg[0].twist.twist.linear.x
Note that the coordinates for linear velocity are vehicle-centered, so only the x-direction linear velocity should be nonzero.

4. Waypoint Updater (Full): Use /traffic_waypoint to change the waypoint target velocities before publishing to /final_waypoints. Your car should now stop at red traffic lights and move when they are green.

There are several helpful methods that you can use:

- get_waypoint_velocity(self, waypoint): gets the linear velocity (x-direction) for a single waypoint.
- set_waypoint_velocity(self, waypoints, waypoint, velocity): sets the linear velocity (x-direction) for a single waypoint in a list of waypoints. Here, waypoints is a list of waypoints, waypoint is a waypoint index in the list, and velocity is the desired velocity.
- distance(self, waypoints, wp1, wp2): Computes the distance between two waypoints in a list along the piecewise linear arc connecting all waypoints between the two. Here, waypoints is a list of waypoints, and wp1 and wp2 are the indices of two waypoints in the list. This method may be helpful in determining the velocities for a sequence of waypoints leading up to a red light (the velocities should gradually decrease to zero starting some distance from the light).

To accomplish this part of the project successfully, you will need to adjust the target velocities for the waypoints leading up to red traffic lights or other obstacles in order to bring the vehicle to a smooth and full stop. You should aim to have a smooth decrease in velocity leading up to the stopping point.

It will be up to you determine what the deceleration should be for the vehicle, and how this deceleration corresponds to waypoint target velocities. As in the Path Planning project, acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3.

Important:
Be sure to limit the top speed of the vehicle to the km/h velocity set by the velocity rosparam in waypoint_loader. Reviewers will test on the simulator with an adjusted top speed.


'''
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

from scipy.spatial import KDTree
import numpy as np

from get_closest_waypoint_idx import get_closest_waypoint_idx_generic # re-use with tl_detector

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

#LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 50 # cf https://knowledge.udacity.com/questions/499973, otherwise car goes out of the road...
# had also to correct Autoware pure_pursuit_core.cpp + .h, relative_angle_threshold_(0.1)
MAX_DECEL = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        ''' Note from project lesson : 
        Once traffic light detection is working properly or you are pulling in the light information from the simulator, 
        you can incorporate the traffic light data into your waypoint updater node. To do this, you will need to add a subscriber 
        for the /traffic_waypoint topic and implement the traffic_cb callback for this subscriber.
        '''
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_wp_idx = -1
        
        # rospy.spin()
        self.loop() 
        ''' 
        We've defined a loop function, because this gives us control over
        the publishing frequency. So we want to target 50 Hz. You could probably get away
        with something little less that 50Hz in this case. The final_waypoints we will be 
        publishing go to the waypoints_follower onto the DBW, which is a piece of code from Autoware, 
        and we believe it is running at 30 Hz.
        So we've got that loop that's constantly happening at 50Hz.
        '''    
    
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                '''if we have self.pose and self.base_waypoints,
                we need to get that closest waypoint. We use this function
                self.get_closest_waypoint_idx() and then we're going to be using
                that KDTree
                '''
                # Get closest waypoint
                # closest_waypoint_idx = self.get_closest_waypoint_idx()
                # Try with more generic function to be reused as well by tl_detector
                closest_waypoint_idx = get_closest_waypoint_idx_generic(x=self.pose.pose.position.x,
                                                                        y=self.pose.pose.position.y,
                                                                        waypoint_tree=self.waypoint_tree,
                                                                        waypoints_2d=self.waypoints_2d,
                                                                        need_next = True)
                
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()
    
    def get_closest_waypoint_idx(self):
        # get coordinates of the car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        ''' Then doing query on the tree base on x and y, ',1' for returning only 1 item
        so the closest point in our KDTree to our query item, the query will return the position
        and also the index, and the index is in the same order as you put in the KDTree'''
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        
        # Check if the closest coordinate is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx -1]
        
        # equation for hyperplane through closes coords
        ''' We use this hyperplane and then do a dot product to see if that's 
        positive or negative, and based on that dot product between those 2 vectors,
        we see if that first wave point is indeed in front of vehicle or not'''
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        
        '''If it's behind the car, then we just ignore it and cut it off.
        And we just take the next one'''
        
        if val > 0 : # means the closest waypoint is behind us
            closest_idx = (closest_idx +1) % len(self.waypoints_2d)
            
        return closest_idx
    
    
    def publish_waypoints(self, closest_idx):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        
    '''
    Notes from project lesson : method generate_lane()
    We're gonna take the waypoints, and update their velocity based on what we want the car to behave.
    So if we have some traffic lights information coming in, we want to slow the car down leading up
    to the stop line in front of the traffic line.
    All we have to do is to update the twist.linear.x
    There are some extra logic in Autoware that helps us to get the car to that speed.
    '''
    def generate_lane(self):
        '''Message type needs to be a lane so we create a new lane object, or new
        lane message, lane header same as base_waypoints header, it does not really matter,
        we're not going to use the header ... and then fill the waypoints for that lane, starting
        by the closes_idx to the next LOOKAHEAD_WPS waypoints '''
        lane = Lane()
        
        #closest_idx = self.get_closest_waypoint_idx()
        
        closest_idx = get_closest_waypoint_idx_generic(x=self.pose.pose.position.x,
                                                       y=self.pose.pose.position.y,
                                                       waypoint_tree=self.waypoint_tree,
                                                       waypoints_2d=self.waypoints_2d,
                                                       need_next = True)
        
        farther_idx = closest_idx + LOOKAHEAD_WPS
        '''Do we have to worry about doing a modular in case it goes over the length of waypoints ?
        we don't because Python slicing is really nice, so if closest index + LOOKAHEAD waypoints
        is bigger than the total length of waypoints, it will just give us the slice from the closest
        to the end of the waypoints'''
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farther_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farther_idx) :
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            
        return lane
    ''' We want to be little careful here, the decelerate_waypoints() will create new waypoints message types here
    Becauce we do not want to modify our original waypoints. Because that message waypoints comes only once, and
    we want to keep the base_waypoints preserved. Otherwise if we start changing modify the base_waypoints, there
    would be problems when we drive back over the same waypoints.
    So we want to create new list of waypoints but use some information from our base_waypoints.
    
    '''
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            
            # Create new waypoint message here. 
            p = Waypoint()
            ''' set pose to base_waypoints pose because position of the waypoint is not going to change anyway and 
            that pose also contains the orientation and it should keep the same as well.
            '''
            p.pose = wp.pose
        
            stop_idx = max(self.stopline_wp_idx - closest_idx -6, 0) # 2 waypoints back from line so front of car stops at line.
            ''' calculate the distance between waypoint index i and waypoint at stop_idx
            distance() will return 0 if i is greater than stop_idx
            '''
            dist = self.distance(waypoints, i, stop_idx)
            '''
            Using SquareRoot function because it is very similar to what is the waypoint_loader code
            vel function of dist, as we get closer (dist decrease) --> vel decreases until 0, and the sqrt()
            will help that. You may want to not use sqrt() because the deceleration will become steep as you
            get close to the stop line.
            So instead of a square root, you could just multiply by a constant. So you get some linear deceleration.
            '''
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # if velocity is small enough, we just set it to 0.
            if vel < 1. :
                vel = 0
            '''sqrt() can become very large as the distance is large, so we do not want to set a very large velocity 
            if we're a long way away from the stop waypoint, we'd rather keep the velocity that was given for the waypoints
            before. So we treat that like the speed limit.
            All the velocity on the waypoints are the speed limit and there's no reason we ever want to be higher than that.
            So we keep the speed limit, and that as the square root becomes smaller than the speed limit, we switch to the 
            square root velocity.
            '''
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        
        return temp
    
    def pose_cb(self, msg):
        # TODO: Implement
        # Just store the car's position
        self.pose = msg # This is called frequently, around 50 Hz
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints 
        ''' store coming waypoints in the object. Latched subscriber, 
         so once the call back is called, it does not send the base_waypoints anymore.
         This is good and ok because the base_waypoints are never changing.
         Basic idea : want to take a chunk of theses waypoints, and use the first 200 that are
         in the front of the car as a reference.
         '''
        
        # Use KDTree data structure to search the closest point in space really efficiently
        if not self.waypoints_2d: 
            ''' Because we want to make sure that self.waypoints_2d is initialized before 
             the subscriber is otherwise could run into risky conditions where the subscriber callback is called 
             before self.waypoints_2d is initialized, otherwise it would not know what to reference
             '''
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
