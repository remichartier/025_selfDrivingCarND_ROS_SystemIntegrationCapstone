#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Notes from Project Lesson : 

This python file processes the incoming traffic light data and camera images. It uses the light classifier to get a color prediction, and publishes the location of any upcoming red lights.

Note: The below section regarding adding in traffic light classification is an optional, stand out suggestion for the project. We of course encourage you to at least make an attempt at a working traffic light classifier, but it is not required for passing the project. You can just pull in the light information directly from the simulator to perform stop and go actions at intersections, if desired.

You can instead just use the traffic light information returned from the simulator to still configure stopping at an intersection line and determining when to again proceed.

You will build both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within tl_detector.py, whereas traffic light classification should take place within ../tl_detector/light_classification_model/tl_classfier.py.

Suggested Order of Project Development
Because you will be writing code across several packages with some nodes depending on messages published by other nodes, we suggest completing the project in the following order:

1. Waypoint Updater Node (Partial): Complete a partial waypoint updater which subscribes to /base_waypoints and /current_pose and publishes to /final_waypoints.
2. DBW Node: Once your waypoint updater is publishing /final_waypoints, the waypoint_follower node will start publishing messages to the/twist_cmd topic. At this point, you have everything needed to build the dbw_node. After completing this step, the car should drive in the simulator, ignoring the traffic lights.
3. Traffic Light Detection: This can be split into 2 parts:
- (Optional) Detection: Detect the traffic light and its color from the /image_color. The topic /vehicle/traffic_lights contains the exact location and status of all traffic lights in simulator, so you can test your output.
- Waypoint publishing: Once you have correctly identified the traffic light and determined its position, you can convert it to a waypoint index and publish it.
'''

''' Additional Notes from the project lesson : 
Traffic Light Detection Node Overview
=====================================
Once the vehicle is able to process waypoints, generate steering and throttle commands, and traverse the course, it will also need stop for obstacles. Traffic lights are the first obstacle that we'll focus on.

The traffic light detection node (tl_detector.py) subscribes to four topics:

/base_waypoints provides the complete list of waypoints for the course.
/current_pose can be used to determine the vehicle's location.
/image_color which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
/vehicle/traffic_lights provides the (x, y, z) coordinates of all traffic lights.
The node should publish the index of the waypoint for nearest upcoming red light's stop line to a single topic:

/traffic_waypoint
For example, if waypoints is the complete list of waypoints, and an upcoming red light's stop line is nearest to waypoints[12], then 12 should be published /traffic_waypoint. This index can later be used by the waypoint updater node to set the target velocity for waypoints[12] to 0 and smoothly decrease the vehicle velocity in the waypoints leading up to waypoints[12].

The permanent (x, y) coordinates for each traffic light's stop line are provided by the config dictionary, which is imported from the traffic_light_config file:

config_string = rospy.get_param("/traffic_light_config")
self.config = yaml.load(config_string)
Your task for this portion of the project can be broken into two steps:

1. Use the vehicle's location and the (x, y) coordinates for traffic lights to find the nearest visible traffic light ahead of the vehicle. This takes place in the process_traffic_lights method of tl_detector.py. You will want to use the get_closest_waypoint method to find the closest waypoints to the vehicle and lights. Using these waypoint indices, you can determine which light is ahead of the vehicle along the list of waypoints.
2. Use the camera image data to classify the color of the traffic light. The core functionality of this step takes place in the get_light_state method of tl_detector.py. There are a number of approaches you could take for this task. One of the simpler approaches is to train a deep learning classifier to classify the entire image as containing either a red light, yellow light, green light, or no light. One resource that's available to you is the traffic light's position in 3D space via the vehicle/traffic_lights topic.
Note that the code to publish the results of process_traffic_lights is written for you already in the image_cb method.

traffic_light_config
This config file contains information about the camera (such as focal length) and the 2D position of the traffic lights's stop line in world coordinates.

Helper Tool in the Simulator
============================
In order to help you acquire an accurate ground truth data source for the traffic light classifier, the Udacity simulator publishes the current color state of all traffic lights in the simulator to the /vehicle/traffic_lights topic in addition to the light location. This state can be used to generate classified images or subbed into your solution to help you work on another single component of the node. The state component of the topic won't be available when running your solution in real life so don't rely on it in the final submission. However, you can still reference this topic in real life to get the 3D world position of the traffic light.

'''

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

""" re-use get_closest_waypoint_idx_generic() from node waypoint_updater
+ to use KDTree for this
"""
from scipy.spatial import KDTree
import numpy as np
import sys
sys.path.append('../waypoint_updater')
from get_closest_waypoint_idx import get_closest_waypoint_idx_generic

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        
        # to use get_closest_waypoint_idx_generic() with KDTree
        self.waypoints_2d = None
        self.waypoints_tree = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        
        """ re-use get_closest_waypoint_idx_generic() from node waypoint_updater
        + to use KDTree for this
        """
        # Use KDTree data structure to search the closest point in space really efficiently
        if not self.waypoints_2d: 
            ''' Because we want to make sure that self.waypoints_2d is initialized before 
            the subscriber is otherwise could run into risky conditions where the subscriber callback is called 
            before self.waypoints_2d is initialized, otherwise it would not know what to reference
            '''
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

        
        
    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        """ Personal notes : 
        - pose comes from ('/current_pose', PoseStamped, self.pose_cb)
        
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
        
        Waypoints in self.waypoints (('/base_waypoints', Lane, self.waypoints_cb))
        
        styx_msgs/Lane
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
        
        Looking for closest waypoint ahead to a pose (x,y) already implemented in some ways 
        via some methods in waypoint_updater node.
        
        
        """
        #TODO implement
        # Get closest waypoint
        # closest_waypoint_idx = self.get_closest_waypoint_idx()
        # Try with more generic function to be reused from waypoint_updater node
        closest_waypoint_idx = get_closest_waypoint_idx_generic(x=pose.pose.position.x,
                                                                y=pose.pose.position.y,
                                                                waypoint_tree=self.waypoint_tree,
                                                                waypoints_2d=self.waypoints_2d)
        
        
        # index of the closest waypoint in self.waypoints
        return closest_waypoint_idx
    
    

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    """ Notes from Project lesson : 
    Your task for this portion of the project can be broken into two steps:

    1. Use the vehicle's location and the (x, y) coordinates for traffic lights to find the nearest visible traffic light ahead of
    the vehicle. This takes place in the process_traffic_lights method of tl_detector.py. You will want to use the
    get_closest_waypoint() method to find the closest waypoints to the vehicle and lights. Using these waypoint indices, you can
    determine which light is ahead of the vehicle along the list of waypoints.
    """
    
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        """ The permanent (x, y) coordinates for each traffic light's stop line are provided by the config dictionary
        ie in self.config
        """
        
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
