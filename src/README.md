Package Overview

The bot_world package provides a simulated world environment for the robot, with launch files to load the world in Gazebo and spawn the robot in it. It includes example world files representing a complex simulation environment.
File Structure

bot_world/
├── worlds/
│   ├── my_world.world       # Custom Gazebo world file
│   ├── (other optional worlds)
├── launch/
│   ├── load_world.launch.py # Loads the Gazebo world and spawns the robot
└── README.md

How to Test This Package

    Build the Workspace:

colcon build
source install/setup.bash

Launch the World:

ros2 launch bot_world load_world.launch.py

    Verify that the Gazebo simulation starts with the specified world and the robot is spawned at the center.

Inspect the World:

    Navigate the Gazebo interface to explore the environment.
    Test the robot's interaction with obstacles or paths.

Check Sensor Data:

    Echo topics like /scan (LiDAR) and /camera/image_raw (RGB camera) to verify sensor data:

        ros2 topic echo /scan
        ros2 topic echo /camera/image_raw

Dependencies

    gazebo_ros_pkgs
    bot_description

Notes

    Ensure bot_description is built and sourced before testing bot_world.
    Customize the world file as needed to match specific scenarios.

