1. Project Overview

This project involves the creation of a robotic system that can be visualized and controlled using ROS 2. The project includes three primary packages:

    bot_description: Contains the URDF model of the robot, launch files for visualization and simulation.
    bot_world: Contains the world files and launch files to set up the robot in a simulated environment.
    bot_control: Contains the logic to read laser scan data, filter it, and control the robot.

2. Installation

Follow these steps to install and test the system:

    Clone the repository to your ROS 2 workspace:

cd ~/bhavish_ws/src
git clone <your-repository-link>

Build the workspace:

    cd ~/bhavish_ws
    colcon build
    source install/setup.bash

3. Package Structure
bot_description

This package contains:

    URDF Model: A description of the robot's structure in URDF format.
    Launch Files:
        rviz.launch.py: Launches RViz for robot visualization.
        spawn.launch.py: Spawns the robot into an empty Gazebo world.
        control.launch.py: Launches the teleoperation functionality using the teleop_twist_keyboard package.

bot_world

This package contains:

    World Files: Descriptions of different simulated environments.
    Launch Files:
        spawn_robot.launch.py: Spawns the robot URDF into the selected world in Gazebo.

bot_control

This package contains:

    C++ Script (reading_laser.cpp): Subscribes to the /scan topic, filters laser scan data from 0 to 120 degrees, and publishes it to /filtered_scan.
    Python Script (reading_laser.py): Similar functionality as the C++ script, written in Python.
    Launch File: Launches the filtering script and RViz for visualization of the filtered data.

4. How to Test
bot_description

    To visualize the robot in RViz:

ros2 launch bot_description rviz.launch.py

To spawn the robot in Gazebo:

ros2 launch bot_description spawn.launch.py

To control the robot using keyboard teleoperation:

    ros2 launch bot_description control.launch.py

bot_world

    To spawn the robot in a Gazebo world:

    ros2 launch bot_world spawn_robot.launch.py

    You can modify the world files in the worlds folder to create new environments.

bot_control

    To run the laser scan filter in Python:

ros2 run bot_control reading_laser.py

To visualize the filtered scan in RViz:

    Open RViz and select the /filtered_scan topic.
    Add a LaserScan display type and set it to the /filtered_scan topic.

To visualize the filtered laser scan data with the launch file:

    ros2 launch bot_control filtered_scan.launch.py

5. Known Issues

    Make sure you have the necessary dependencies installed for teleop_twist_keyboard.
    If there are issues with Gazebo or RViz, verify the setup of your ROS 2 workspace and source the environment correctly (source install/setup.bash).

6. Contributing

Feel free to open an issue or submit a pull request if you'd like to contribute improvements or bug fixes.
7. License

This project is licensed under the MIT License - see the LICENSE file for details.
