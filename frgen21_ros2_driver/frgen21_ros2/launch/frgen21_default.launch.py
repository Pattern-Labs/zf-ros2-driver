from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frgen21_ros2',
            executable='frgen21_node',
            name='frgen21_node',
            parameters=[{'input_port': 60001},
                        # {'multicast_ip': '239.168.100.30'},
                        # {'host_ip': '127.0.0.1'},
                        {'frame_id': 'frgen21'},
                        {'publish_pc': True},
                        {'publish_scan': True},
                        {'publish_info': True},
                        {'publish_frame': False},
                        {'topic_pc': '/frgen21/pointcloud'},
                        {'topic_scan': '/frgen21/scan'},
                        {'topic_info': '/frgen21/info'},
                        {'topic_frame': '/frgen21/frame'}
                        ]
        ),
        Node(
            package='frgen21_ros2',
            executable='frgen21_service',
            name='frgen21_service',
            parameters=[{'name_service': '/frgen21/input'},
                        {'topic_info': '/frgen21/info'},
                        {'sensor_ip': '192.168.50.18'},
                        {'sensor_input_port': 60002},
                        {'host_output_port': 60002}
                        ]
        )
    ])
