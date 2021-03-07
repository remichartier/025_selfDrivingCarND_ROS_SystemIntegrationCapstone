#!/usr/bin/env python

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

'''
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # TODO: Add other member variables you need below
        self.pose = none
        self.base_waypoints = none
        self.waypoints_2d = none
        self.waypoints_tree = none
        
        # rospy.spin()
        self.loop() ''' We've defined a loop function, because this gives us control over
        the publishing frequency. So we want to target 50 Hz. You could probably get away
        with something little less that 50Hz in this case. The final_waypoints we will be 
        publishing go to the waypoints_follower onto the DBW, which is a piece of code from Autoware, 
        and we believe it is running at 30 Hz.
        So we've got that loop that's constantly happening at 50Hz.
        '''    
    
    def loop(self):
        rate = rospy.rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                '''if we have self.pose and self.base_waypoints,
                we need to get that closest waypoint. We use this function
                self.get_closest_waypoint_idx() and then we're going to be using
                that KDTree
                '''
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
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
        '''Message type needs to be a lane so we create a new lane object, or new
        lane message, lane header same as base_waypoints header, it does not really matter,
        we're not going to use the header ... and then fill the waypoints for that lane, starting
        by the closes_idx to the next LOOKAHEAD_WPS waypoints '''
        lane = lane()
        lane.header = self.base_waypoints.header
        '''Do we have to worry about doing a modular in case it goes over the length of waypoints ?
        we don't because Python slicing is really nice, so if closest index + LOOKAHEAD waypoints
        is bigger than the total length of waypoints, it will just give us the slice from the closest
        to the end of the waypoints'''
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
        
    def pose_cb(self, msg):
        # TODO: Implement
        # Just store the car's position
        self.pose = msg # This is called frequently, around 50 Hz
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints ''' store coming waypoints in the object. Latched subscriber, 
         so once the call back is called, it does not send the base_waypoints anymore.
         This is good and ok because the base_waypoints are never changing.
         Basic idea : want to take a chunk of theses waypoints, and use the first 200 that are
         in the front of the car as a reference.
        '''
        # Use KDTree data structure to search the closest point in space really efficiently
        if not self.waypoints_2d: ''' Because we want to make sure that self.waypoints_2d is initialized before 
             the subscriber is otherwise could run into risky conditions where the subscriber callback is called 
             before self.waypoints_2d is initialized, otherwise it would not know what to reference'''
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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
