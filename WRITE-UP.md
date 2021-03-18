# Udacity Self-Driving Car Nanodegree : CarND-System-Integration / Capstone final project : Programming a Real Self-Driving Car (using ROS for system integration)

Running Code Criteria | Criteria to meet specifications
-------------------- | -------------------------------
The code is built successfully and connects to the simulator.|Running `catkin_make`, source `devel/setup.sh` and `roslaunch launch/styx`.launch within the ros directory results in no errors and allows the program to connect to the simulator.

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
