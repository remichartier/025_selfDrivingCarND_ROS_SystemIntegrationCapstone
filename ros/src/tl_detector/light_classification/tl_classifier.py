# -*- coding: utf-8 -*-
'''
Notes from project lesson : 

This file contains the TLClassifier class. You can use this class to implement traffic light classification. For example, the get_classification method can take a camera image as input and return an ID corresponding to the color state of the traffic light in the image. Note that it is not required for you to use this class. It only exists to help you break down the classification problem into more manageable chunks. Also note that Carla currently has TensorFlow 1.3.0 installed. If you are using TensorFlow, please be sure to test your code with this version before submission.
'''

from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
