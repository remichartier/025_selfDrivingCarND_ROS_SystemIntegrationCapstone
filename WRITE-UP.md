# Udacity Self-Driving Car Nanodegree : CarND-System-Integration / Capstone final project : Programming a Real Self-Driving Car (using ROS for system integration)

Running Code Criteria | Criteria to meet specifications
-------------------- | -------------------------------
The code is built successfully and connects to the simulator.|Running `catkin_make`, source `devel/setup.sh` and `roslaunch launch/styx`.launch within the ros directory results in no errors and allows the program to connect to the simulator.

No problem running catkin_make and then roslaunch : 
```
(venv) root@81262c4e00e7:/home/workspace/CarND-Capstone/ros# roslaunch launch/styx.launch 
... logging to /root/.ros/log/0b207c76-8876-11eb-a3ef-0242ac110002/roslaunch-81262c4e00e7-719.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://81262c4e00e7:41681/

SUMMARY
========

PARAMETERS
 * /dbw_node/accel_limit: 1.0
 * /dbw_node/brake_deadband: 0.2
 * /dbw_node/decel_limit: -5.0
 * /dbw_node/fuel_capacity: 0.0
 * /dbw_node/max_lat_accel: 3.0
 * /dbw_node/max_steer_angle: 8.0
 * /dbw_node/steer_ratio: 14.8
 * /dbw_node/vehicle_mass: 1080.0
 * /dbw_node/wheel_base: 3
 * /dbw_node/wheel_radius: 0.335
 * /pure_pursuit/linear_interpolate_mode: True
 * /rosdistro: kinetic
 * /rosversion: 1.12.14
 * /traffic_light_config: <...>
 * /waypoint_loader/path: /home/workspace/C...
 * /waypoint_loader/velocity: 40

NODES
  /
    dbw_node (twist_controller/dbw_node.py)
    pure_pursuit (waypoint_follower/pure_pursuit)
    styx_server (styx/server.py)
    tl_detector (tl_detector/tl_detector.py)
    unity_simulator (styx/unity_simulator_launcher.sh)
    waypoint_loader (waypoint_loader/waypoint_loader.py)
    waypoint_updater (waypoint_updater/waypoint_updater.py)

auto-starting new master
process[master]: started with pid [729]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 0b207c76-8876-11eb-a3ef-0242ac110002
process[rosout-1]: started with pid [742]
started core service [/rosout]
process[styx_server-2]: started with pid [745]
process[unity_simulator-3]: started with pid [760]
What is the full path to your Unity simulator?
process[dbw_node-4]: started with pid [767]
process[waypoint_loader-5]: started with pid [773]
process[pure_pursuit-6]: started with pid [855]
process[waypoint_updater-7]: started with pid [873]
process[tl_detector-8]: started with pid [912]
```

Control and Planning Criteria | Criteria to meet specifications
-------------------- | -------------------------------
Waypoints are published to plan Carla’s route around the track. | Waypoints should be published to `/final_waypoints` to plan the vehicle’s path around the track. No unnecessary moves (excessive lane changes, unnecessary turning, unprompted stops) should occur. As in the Path Planning project, acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3. Be sure to limit the top speed of the vehicle to the km/h velocity set by the velocity rosparam in `waypoint_loader`.

- For publication of `/final_waypoints`, I had to modify the original value of `LOOKAHEAD_WPS`, initially set to 250 waypoints. However this number of waypoints make the steering and yaw_controller impossible to control the steering, the vehicle would drift and oscillate left and right around the waypoints lane until loosing steering control and drive out of the road.
  - I had to set this constant to 50 waypoints. Even something like 75 waypoints would cause the vehicle to drive away off the road later on in the track. I really do not know why yet. 

- lane changes / unnecessary turning : Initial steering control using Autoware code (`waypoint_follower`) would be very harsh, due to steering error threshold being too loose. (5.0 radians). I reduced it to `relative_angle_threshold_(0.1)` in order to trigger more quickly the steering PID control, in Autoware code `pure_pursuit_core.h`
  - I also removed the distance threshold before Autoware code would start to correct steering drift, in `pure_pursuit_core.h`

```
  // if (displacement < displacement_threshold_ && relative_angle < relative_angle_threshold_)
  if (relative_angle < relative_angle_threshold_)
```

Control and Planning Criteria | Criteria to meet specifications
-------------------- | -------------------------------
Controller commands are published to operate Carla’s throttle, brake, and steering. | `dbw_node.py` has been implemented to calculate and provide appropriate throttle, brake, and steering commands. The commands are published to `/vehicle/throttle_cmd`, `/vehicle/brake_cmd` and `/vehicle/steering_cmd`, as applicable.

- Nothing special to comment on that. It is working as expected.

Successful NavigationCriteria | Criteria to meet specifications
-------------------- | -------------------------------
Successfully navigate the full track more than once. | The vehicle is able to complete more than one full loop of the track without running off road or any other navigational issues (incorrect turns, random stops, teleportation, etc.).

- Nothing special to comment on that. It is working as expected. The vehicle navigates until the end of the waypoints.

## Stopping at the traffic line stop

- The stop line positions, provided by the `/traffic_light_config`, does not seem accurate, because on some traffic lights, if following those informations, vehicle stops sometimes few meters after the stop line position. 
- I fixed this issue by stopping the car 13 way points before the closest waypoint of the stop line position. So sometimes the car would stop 1 or 2 meters before the stop line position, or sometimes it would stop at the line position. I think this innacuracy is largely due to lack of accuracy of the `/traffic_light_config` line position info.

cf in `waypoint_updater.py` : Project lesson was suggesting to stop the car 2 waypoints before stop line. I had to increase to 12 waypoints in order to make sure it would stop before the stop line of every traffic line on this track and simulator. 
```
stop_idx = max(self.stopline_wp_idx - closest_idx -12, 0) # 2 waypoints back from line so front of car stops at line.
```

## Additional improvements to be done in future : 

### Traffic Light Detection

- I skipped it as it was optional. I will come back to it after I graduate. So for time being, traffic light info is collected via `/vehicle/traffic_lights` topic/msg, and used to retrive traffic light colors instead of using a classification model based on camera images.

### Object Detection

- I skipped it as it was optional. I will come back to it after I graduate.

-
