import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription,  TimerAction

def generate_launch_description():
    robot_localization_params = os.path.join(get_package_share_directory('robot-nav'),'config','ekf.yaml')
    package_name='robot-nav'

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_params]
    )


    launch_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch/joystick.launch.py'))
    )

    map_pose_publisher_node = Node(
        package='tf2_web_publisher',
        executable='map_pose_publisher',
        name='map_pose_publisher',
        output='screen'
    )

    camera_to_map_transformer_node = Node(
        package='tf2_web_publisher',
        executable='camera_to_map_transformer',
        name='camera_to_map_transformer',
        output='screen'
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        robot_localization_node,
        launch_joystick,
        map_pose_publisher_node,
        camera_to_map_transformer_node,
        rosbridge_node
    ])