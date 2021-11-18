from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lidar_tf = Node(package = "tf2_ros", 
                   executable = "static_transform_publisher",
                   arguments = ["0", "0", "0", "0", "0", "0", "map", "lidar"])

    danger_zone_node = Node(
        package='danger_zone',
        namespace='gaia',
        executable='danger_zone',
        name='danger_zone',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([lidar_tf, danger_zone_node])
