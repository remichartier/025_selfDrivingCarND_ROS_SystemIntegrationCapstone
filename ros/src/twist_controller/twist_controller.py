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
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed=0.1, max_lat_accel, max_steer_angle)
        
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
        self.vel_lps = LowPassFilter(tau, ts)
        
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
        
        # return 1., 0., 0. # if want to Sanity check and check the car is moving forward
