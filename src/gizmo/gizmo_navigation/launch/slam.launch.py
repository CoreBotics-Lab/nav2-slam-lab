import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    # Launch arguments
    launch_arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_slam_params = os.path.join(
        gizmo_navigation_dir,
        'config',
        'slam_toolbox.yaml'
    )
    launch_arg_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the ROS 2 parameters file for slam_toolbox'
    )
    slam_params_file = LaunchConfiguration('slam_params_file')

    default_rviz_config = os.path.join(
        gizmo_navigation_dir,
        'rviz',
        'slam.rviz'
    )
    launch_arg_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # SLAM Toolbox Node (Online Asynchronous)
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz2 Node with SLAM layout
    start_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        launch_arg_use_sim_time,
        launch_arg_slam_params,
        launch_arg_rviz_config,
        start_async_slam_toolbox_node,
        # start_rviz2_node
    ])

