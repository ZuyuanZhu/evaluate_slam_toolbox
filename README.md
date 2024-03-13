Evaluation scheme: 

  

1. Environment Setup 

Group Foxy: Ubuntu 20.04 + ROS2 foxy + slam_toolbox (foxy branch, commit:?) + turlterbot3-foxy  

Group Humble: Ubuntu 22.04 + ROS2 humble + slam_toolbox (humble branch, commit: ?) + turlterbot3-humble   

    Maps to be tested:   

        Default nav2 map 

        Tiny object test map (rvc pkg) 

        turtlebot3_stage_4.world (rvc pkg) 

2. Goal Positions 

Define a set of goal positions throughout the map. These positions cover different areas and include various challenges such as close proximity to obstacles, open areas, and narrow passages.  

3. Navigation to Goal Positions 

For each goal position, use the ROS2 navigation stack to command the robot to navigate to the location. Publish a goal pose to the /goal_pose topic. The robot starts from a known initial position for each trial to maintain consistency.  

ros2 topic pub /goal_pose geometry_msgs/PoseStamped "header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: POS_X, y: POS_Y, z: 0.0}, orientation: {w: 1.0}}"   

Use different positions (POS_X, POS_Y) for evaluation.  

4. Evaluate Pose Error 

Pose error evaluation involves comparing the robot's estimated pose (as reported by the SLAM system) when it reaches the goal position with the actual goal pose. This comparison can be done using the following metrics:  

Positional Error: The Euclidean distance between the estimated position and the actual goal position.  

Orientation Error: The angular difference between the estimated orientation and the actual goal orientation. This can be calculated using quaternion angle representations.  

  
