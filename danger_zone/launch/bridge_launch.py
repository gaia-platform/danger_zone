from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lidar_tf = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "lidar"])
    
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

    return LaunchDescription([lidar_tf, unity_bridge_node])
