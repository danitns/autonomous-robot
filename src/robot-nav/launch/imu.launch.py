from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    imu_filter_params = os.path.join(get_package_share_directory('robot-nav'),'config','imu_filter.yaml')
    robot_localization_params = os.path.join(get_package_share_directory('robot-nav'),'config','ekf.yaml')

    imu_node_raw = Node(
            package='lsm9ds1_ros2_driver',
            executable='imu_data_publisher',
            parameters=[],
         )

    imu_filter_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[imu_filter_params],
         )

    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_params]
        )

    return LaunchDescription([
        imu_node_raw,
        imu_filter_node,
        robot_localization_node
    ])