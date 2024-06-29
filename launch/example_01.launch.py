from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the directory where the params are stored
    package_share_directory = get_package_share_directory('imarker_pose_mockups')

    # Define paths to the parameter files
    params1_path = os.path.join(package_share_directory, 'config', 'example/example_01_goal.config.yaml')
    params2_path = os.path.join(package_share_directory, 'config', 'example/example_01_start.config.yaml')


    rviz_path = os.path.join(package_share_directory, 'config', 'rviz2/example_01.rviz')

    declare_rviz_config = DeclareLaunchArgument('rvizconfig', default_value=rviz_path, description='Path to the RViz config file')
    rviz_config_file = LaunchConfiguration('rvizconfig')

    # Add the RViz node with the specified config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


    return LaunchDescription([
        Node(
            package='imarker_pose_mockups',
            executable='InteractiveMarkerPointXYZNode',
            name='interactive_marker_node_goal',
            parameters=[params1_path]
        ),
        Node(
            package='imarker_pose_mockups',
            executable='InteractiveMarkerPointXYZNode',
            name='interactive_marker_node_start',
            parameters=[params2_path]
        ),
        declare_rviz_config,
        rviz_node,
    ])