import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch description
    ld = LaunchDescription()

    # Launch pico_visualizer node
    pico_visualizer_node = Node(
        package='pico_visualizer',
        executable='pico_visualizer',
        output='screen'
    )
    ld.add_action(pico_visualizer_node)

    # Launch tf2_static_transform_publisher node
    tf_static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'base_link'],
        output='screen'
    )
    ld.add_action(tf_static_transform_publisher_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()