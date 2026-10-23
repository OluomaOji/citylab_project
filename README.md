# citylab_project

ROS 2 patrol behavior for the FastBot robot (package: `robot_patrol`).

## Behavior
The robot drives forward at 0.1 m/s. If an obstacle is closer than 0.35 m
within +/-20 deg of the front, it slows to 0.05 m/s and turns toward the
farthest valid laser ray in the front 180 deg (left = positive angular.z,
right = negative).

## Branches
- `main`: simulation 
- `real-robot`: real robot

## Build and run
    cd ~/ros2_ws
    colcon build --packages-select robot_patrol
    source install/setup.bash
    ros2 launch robot_patrol start_patrolling.launch.py

## Parameters
`obstacle_distance` (0.35), `linear_speed` (0.1), `turn_speed` (0.5)