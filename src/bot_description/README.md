Package Overview

The bot_description package contains the URDF model and associated configurations for a robot. It includes launch files to visualize the robot in RViz, spawn it in Gazebo, and control it via teleoperation. The robot is designed as a differential drive system with LiDAR and an RGB camera.
File Structure

bot_description/
├── launch/
│   ├── rviz.launch.py        # Launches the robot in RViz
│   ├── spawn.launch.py       # Spawns the robot in an empty Gazebo world
│   ├── control.launch.py     # Enables teleoperation using keyboard commands
├── urdf/
│   └── robot.urdf.xacro      # Robot description file in XACRO format
├── meshes/
│   └── (optional) Robot meshes
├── config/
│   └── (optional) Configuration files for sensors/plugins
└── README.md

How to Test This Package

    Build the Workspace:

colcon build
source install/setup.bash

Visualize in RViz:

ros2 launch bot_description rviz.launch.py

    Ensure RViz opens with the robot displayed.
    Use the Transform and RobotModel displays to inspect the robot.

Spawn in Gazebo:

ros2 launch bot_description spawn.launch.py

    Check that the robot appears in the Gazebo simulation.

Teleoperate the Robot:

ros2 launch bot_description control.launch.py

    Open a terminal for teleoperation and issue commands to control the robot.
    Verify that velocity commands are being published to the /cmd_vel topic and that the robot responds in Gazebo.

Inspect Topics:

    Echo the /cmd_vel topic to confirm velocity messages:

        ros2 topic echo /cmd_vel

Dependencies

    gazebo_ros_pkgs
    teleop_twist_keyboard
    rviz2