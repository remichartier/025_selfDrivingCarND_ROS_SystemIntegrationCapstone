#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scipy.spatial import KDTree
import numpy as np

""" defining generic get_closest_waypoint_idx() in order to re-use it for 
    light detection as well

    Pre-requisites : 

        from scipy.spatial import KDTree
        import numpy as np

        method __init__() must define : 
            self.waypoints_2d = None
            self.waypoints_tree = None

        method waypoints_cb() must include : 

            # Use KDTree data structure to search the closest point in space really efficiently
            if not self.waypoints_2d: 
                ''' Because we want to make sure that self.waypoints_2d is initialized before 
                 the subscriber is otherwise could run into risky conditions where the subscriber callback is called 
                 before self.waypoints_2d is initialized, otherwise it would not know what to reference
                 '''
                self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
                self.waypoint_tree = KDTree(self.waypoints_2d)


    Methods inputs : 
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y   

    Way to call this overload function : 

    get_closest_waypoint_idx_generic(x=self.pose.pose.position.x,
                             y=self.pose.pose.position.y,
                             waypoint_tree=self.waypoint_tree,
                             waypoints_2d=self.waypoints_2d)

    If working --> can place in a separate file and use it both for waypoint_updater
    and for tl_detector nodes.
"""

def get_closest_waypoint_idx_generic(x, y, waypoint_tree, waypoints_2d, need_next):
    # get coordinates of the car
    #x = self.pose.pose.position.x
    #y = self.pose.pose.position.y
    ''' Then doing query on the tree base on x and y, ',1' for returning only 1 item
        so the closest point in our KDTree to our query item, the query will return the position
        and also the index, and the index is in the same order as you put in the KDTree'''
    closest_idx = waypoint_tree.query([x,y],1)[1]

    if need_next:
        # Check if the closest coordinate is ahead or behind vehicle
        closest_coord = waypoints_2d[closest_idx]
        prev_coord = waypoints_2d[closest_idx -1]

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
            closest_idx = (closest_idx +1) % len(waypoints_2d)

    return closest_idx