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

    • We are provided with a Yaw controller. For helping with the steering.
    • So we can get steering commands from the Yaw controller, and you need to put in the wheel_base, steering ratio, 
    • 0.1 is the lowest speed of the car in meters per second.
    • And then the max lateral acceleration and the max steering angle.
    • All of those are parameters that we passed in dbw.py so you just need to forward them to the yaw controller.
'''

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,decel_limit,
                 accel_limit, wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
