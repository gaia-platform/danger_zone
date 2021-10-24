import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# TODO: does not launch the Unity simulator yet.
def generate_launch_description():
    rosbag2_params = os.path.join(
        get_package_share_directory('danger_zone'),
        'param',
        'rosbag2_snapshot_topics.params.yaml'
    )

    unity_bridge_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_bridge',
        emulate_tty=True,
        parameters=[
            {'ROS_IP': '127.0.0.1'},
            {'ROS_TCP_PORT': 10000},
        ]
    )

    danger_zone_node = Node(
        package='danger_zone',
        namespace='gaia',
        executable='danger_zone',
        name='danger_zone',
        output='screen',
        emulate_tty=True
    )

    snapshotter_node = Node(
        package='rosbag2_snapshot',
        namespace='gaia',
        executable='snapshotter',
        name='snapshotter',
        output='screen',
        emulate_tty=True,
        parameters = [rosbag2_params]
    )

    return LaunchDescription([unity_bridge_node, danger_zone_node, snapshotter_node])
