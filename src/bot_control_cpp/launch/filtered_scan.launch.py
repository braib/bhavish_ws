import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the rviz config file
    pkg_share = get_package_share_directory('bot_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')

    return LaunchDescription([
        # Launch the reading_laser node to filter laser scan data
        Node(
            package='bot_control_cpp',  # Package name
            executable='reading_laser',  # Executable name
            name='reading_laser_node',  # Node name
            output='screen',  # Output to the screen
        ),

        # Launch rviz to visualize the filtered laser scan data
        Node(
            package='rviz2',  # RViz package
            executable='rviz2',  # Executable to launch rviz
            name='rviz2',  # Node name for rviz
            output='screen',  # Output to the screen
            arguments=['-d', default_rviz_config_path],  # Use the default rviz config
        ),
    ])
