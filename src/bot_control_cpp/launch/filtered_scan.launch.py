import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('bot_control_cpp')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')

    return LaunchDescription([

        Node(
            package='bot_control_cpp',  
            executable='reading_laser', 
            name='reading_laser_node',  
            output='screen', 
        ),

        Node(
            package='rviz2',  
            executable='rviz2',  
            name='rviz2', 
            output='screen',  
            arguments=['-d', default_rviz_config_path], 
        ),
    ])
