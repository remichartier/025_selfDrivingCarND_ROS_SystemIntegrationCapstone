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


Control and Planning Criteria | Criteria to meet specifications
-------------------- | -------------------------------
Controller commands are published to operate Carla’s throttle, brake, and steering. | `dbw_node.py` has been implemented to calculate and provide appropriate throttle, brake, and steering commands. The commands are published to `/vehicle/throttle_cmd`, `/vehicle/brake_cmd` and `/vehicle/steering_cmd`, as applicable.

Successful NavigationCriteria | Criteria to meet specifications
-------------------- | -------------------------------
Successfully navigate the full track more than once. | The vehicle is able to complete more than one full loop of the track without running off road or any other navigational issues (incorrect turns, random stops, teleportation, etc.).


- Utilize a detection and classification model for stop lights. The model should appropriately detect and classify stop lights at intersections at least 80% of the time. Make sure to provide instructions for obtaining the necessary model used, as well as any additional necessary dependencies and setup.
- Waypoints and controls are adjusted based on detections. When approaching a red light, the vehicle should slow and come to a complete stop (if the light is still red). When the light switches back to green, the vehicle should accelerate back up to the desired speed. When approaching a green light, the vehicle should continue normally on its path.
- Include a write-up concerning different models you tried out for the stop light detection, and results on those models.
- Include a write-up concerning any data gathering efforts you performed for the stop lights, as well as any data augmentation.
- Get as close to 100% accuracy as you can with stop light detection and classification - without just overfitting the simulator!

Additional improvements to be done in future : 
-
-
-
