from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    declared_arguments = []

    odom_to_tf_node = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("odom_to_tf_ros2"), 'config', 'odom_to_tf.yaml')],
    )
    
    nodes = [
     odom_to_tf_node,
    ]

    return LaunchDescription(declared_arguments + nodes)