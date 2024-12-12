import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='bot_description').find('bot_description')
    pkg_share_rviz = launch_ros.substitutions.FindPackageShare(package='bot_control_cpp').find('bot_control_cpp')

    default_model_path = os.path.join(pkg_share, 'urdf/my_robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share_rviz, 'rviz/config.rviz')

    world_path=os.path.join(get_package_share_directory('bot_world'), 'worlds/home1.world')
    
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    filtered_scan = launch_ros.actions.Node(
        package='bot_control_cpp',  
        executable='reading_laser', 
        name='reading_laser_node',  
        output='screen', 
    )


    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_first_robot', '-topic', 'robot_description'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        filtered_scan,
        rviz_node
    ])

