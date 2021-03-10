# -*- coding: utf-8 -*-

'''
Notes from project lesson :
===========================

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter that you can use in your implementation. The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.

Additional notes from project lesson : 
======================================
- This file contains a stub of the Controller class. 
- You can use this class to implement vehicle control. 
- For example, the control method can take twist data as input and return throttle, brake, and steering values. 
- Within this class, you can import and use the provided pid.py and lowpass.py if needed for acceleration, and yaw_controller.py for steering. 
- Note that it is not required for you to use these, and you are free to write and import other controllers.

 
'''
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,decel_limit,
                 accel_limit, wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        '''
        • We are provided with a Yaw controller. For helping with the steering.
        • So we can get steering commands from the Yaw controller, and you need to put in the wheel_base, steering ratio, 
        • 0.1 is the lowest speed of the car in meters per second.
        • And then the max lateral acceleration and the max steering angle.
        • All of those are parameters that we passed in dbw.py so you just need to forward them to the yaw controller.
        '''
        # Yaw Controller :
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        # PID Controller for the Throttle : 
        '''
        This parameters for the throttle controller were determined completely experimentally
        from the project lesson video, just testing out what worked
        '''
        Kp = 0.3
        Ki = 0.1
        Kd = 0.0
        minThrottle = 0.0
        maxThrottle = 0.2
        self.throttle_controller = PID(Kp, Ki, Kd, mn=minThrottle, mx=maxThrottle)
        
        '''
        There is a low-pass filter as well.
        Created because the velocity that's coming in over the messages is kind of noisy
        So this low-pass filter is filtering out all of the high frequency noise in 
        the velocity
        '''
        tau = 0.5 # 1/(2*Pi*Tau) = Cut Off Frequency
        ts = 0.2 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        
        ''' 
        Things that we might need or we might not need
        accel_limit / decel_limit set in a configuration file 
        And also as ones of the ROS params cf class DBWNode() / __init__() in dbw_node.py
        They are not pre-set because determined by the car itself, they are more just 
        comfort parameters
        '''
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
        
        
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        '''
        Called by dbw_node.py (loop() method as : 
        self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                   self.dbw_enabled,
                                                                                   self.linear_vel,
                                                                                   self.angular_vel)
        We create instance of this Controller class above and then we call the control() method
        That's 50Hz basically.
        First thing we're checking is if dbw_enabled or not, because can turn drive by wire of
        the car on and off.
        If you're using a PID controller, and you have an integral term, and you don't turn
        the PID controller Off, then you'll be accumulating error, and that will be bad
        because, then when we'll turn the DbW Node back on, you'll have accumulated all this
        error and the car might do something really erratic.
        '''
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        ''' Otherwise, look at current velocity, and steering we're getting from
        this yaw_controller and we're checking out this velocity error, so where
        do we want to be versus where we're currently at.
        We have this last velocity as well.
        '''
        
        current_vel = self.vel_lpf.filt(current_vel)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        # Here we start doing some physics
        
        self.current_time = rospy.get_time()
        sample_time = self.current_time - self.last_time
        self.last_time = self.current_time
        
        # For throttle, we're stepping through our throttle controller using the PID controller
        throttle = self.throttle_controller.step(vel_error, sample_time)
        
        ''' Initially we're setting brake to 0. And now we're doing a check here.
        If our target linear velocity (val) is 0, and we're going very slow,
        So our current velocity is less than 0.1, we should probably be trying to stop.
        So what we're going to do is set throttle to 0 and apply a lot of brake.
        '''
        '''Note
        In the walkthrough, only 400 Nm of torque is applied to hold the vehicle stationary. This turns out to be slightly less than
        the amount of force needed, and Carla will roll forward with only 400Nm of torque. To prevent Carla from moving you should
        apply approximately 700 Nm of torque.
        '''
        brake = 0
        
        if linear_vel == 0 and current_vel < 0.1 :
            throttle = 0
            brake = 400 #400 # N * m; to hold the car in place, If we are stopped at a light
            # Acceleration - 1m/s2
            '''
            Or else throttle also really small and velocity error < 0
            which means we're going faster than we want to be, ie faster than our target velocity,
            And our PID is letting up on the throttle but we want to slow down, throttle < 0.1,
            This time we're not just hard coding the break to 400Nm, but we're using deceleration
            (ie the amount we want to decelerate) *
            vehicle_mass * wheel_radius, that gives us the Torque that we want.
            vehicle_mass in kilograms, wheel_radius in meters, 
            '''
        elif throttle < 0.1 and vel_error < 0 :
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m
        
        # return 1., 0., 0. # if want to Sanity check and check the car is moving forward
        return throttle, brake, steering
