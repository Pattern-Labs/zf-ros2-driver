import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    urdf = os.path.join(
        get_package_share_directory('frgen21_ros2'), 'urdf', 'frgen21.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # launch sensors_frgen21 launch file (start frgen21_node for all sensors)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('frgen21_ros2'),
                    'launch',
                    'frgen21_default.launch.py'
                ])
            ])
        ),

        # launch robot_state_publisher with the URDF file
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # start rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('frgen21_ros2'),
                    'rviz',
                    'frgen21.rviz'
            ])]
        ),
    ])
