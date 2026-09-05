import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    # 1. Declare Launch Arguments
    launch_arg_use_sim_time = DeclareLaunchArgument(
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
    launch_arg_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to the map YAML file to load'
    )
    map_file = LaunchConfiguration('map')

    default_nav2_params = os.path.join(
        gizmo_navigation_dir,
        'config',
        'nav2.yaml'
    )
    launch_arg_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to the Nav2 parameters YAML file'
    )
    params_file = LaunchConfiguration('params_file')

    launch_arg_run_rviz2 = DeclareLaunchArgument(
        'run_rviz2',
        default_value='true',
        description='Run RViz2 if true'
    )
    run_rviz2 = LaunchConfiguration('run_rviz2')

    default_rviz_config = os.path.join(
        gizmo_navigation_dir,
        'rviz',
        'nav2.rviz'
    )
    launch_arg_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz configuration file'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # 2. Lifecycle nodes lists (Standard Nav2 separation)
    localization_nodes = [
        'map_server',
        'amcl'
    ]

    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    # 3. Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_file, 'use_sim_time': use_sim_time}
        ]
    )

    # 4. AMCL Node (Localization)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 5. Controller Server Node (DWB Local Planner)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odometry/filtered')
        ]
    )

    # 6. Planner Server Node (Global Path Planner)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 7. Behavior Server Node (Recoveries)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 8. BT Navigator Node (Behavior Tree Orchestrator)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 9. Waypoint Follower Node
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 10. Lifecycle Managers (Separate managers for Localization and Navigation)
    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': localization_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': navigation_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    # 11. RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_rviz2)
    )

    return LaunchDescription([
        launch_arg_use_sim_time,
        launch_arg_map,
        launch_arg_params_file,
        launch_arg_run_rviz2,
        launch_arg_rviz_config,
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_localization_node,
        lifecycle_manager_navigation_node,
        rviz2_node
    ])
