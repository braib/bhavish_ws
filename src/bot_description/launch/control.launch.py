from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    teleop_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        teleop_node
    ])
