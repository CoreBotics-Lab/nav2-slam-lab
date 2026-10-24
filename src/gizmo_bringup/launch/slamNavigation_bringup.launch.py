import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    # 1. Declare Launch Arguments
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Whether to run SLAM (true) or Localization (false)'
    )
    slam = LaunchConfiguration('slam')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_map_file = os.path.join(
        gizmo_navigation_dir,
        'maps',
        'simple_world_map.yaml'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to the map YAML file to load (for localization)'
    )
    map_file = LaunchConfiguration('map')

    default_nav2_params = os.path.join(
        gizmo_navigation_dir,
        'config',
        'nav2.yaml'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to the Nav2 parameters YAML file'
    )
    params_file = LaunchConfiguration('params_file')

    default_slam_params = os.path.join(
        gizmo_navigation_dir,
        'config',
        'slam_toolbox.yaml'
    )
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the ROS 2 parameters file for slam_toolbox'
    )
    slam_params_file = LaunchConfiguration('slam_params_file')

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    autostart = LaunchConfiguration('autostart')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    default_rviz_config = os.path.join(
        gizmo_navigation_dir,
        'rviz',
        'nav2.rviz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz configuration file'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Whether to use composed Nav2 bringup'
    )
    use_composition = LaunchConfiguration('use_composition')

    # 2. Paths to gizmo_navigation building block launch files
    slam_launch_path = os.path.join(gizmo_navigation_dir, 'launch', 'slam.launch.py')
    localization_launch_path = os.path.join(gizmo_navigation_dir, 'launch', 'localization.launch.py')
    navigation_launch_path = os.path.join(gizmo_navigation_dir, 'launch', 'navigation.launch.py')

    # 3. Conditional GroupActions:
    # If slam == true: Launch SLAM Toolbox (+ Map Saver Server)
    slam_group = GroupAction(
        condition=IfCondition(slam),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_path),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file,
                    'autostart': autostart
                }.items()
            )
        ]
    )

    # If slam == false: Launch Localization (Map Server + AMCL)
    localization_group = GroupAction(
        condition=UnlessCondition(slam),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch_path),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_file,
                    'params_file': params_file,
                    'autostart': autostart
                }.items()
            )
        ]
    )

    # Always Launch Navigation (Controller, Planner, Behaviors, BT Navigator, Waypoint Follower)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition
        }.items()
    )

    # RViz2 Node (optional, defaults to true)
    rviz_group = GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        slam_arg,
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        slam_params_file_arg,
        autostart_arg,
        use_rviz_arg,
        rviz_config_arg,
        use_composition_arg,
        slam_group,
        localization_group,
        navigation_launch,
        rviz_group
    ])

