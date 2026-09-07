import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    # 1. Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    autostart = LaunchConfiguration('autostart')

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Whether to use composed bringup (runs in single container for high reliability)'
    )
    use_composition = LaunchConfiguration('use_composition')

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='Name of the container for composable nodes'
    )
    container_name = LaunchConfiguration('container_name')

    # 2. Lifecycle nodes for Navigation
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    remappings = [
        ('cmd_vel', '/cmd_vel'),
        ('odom', '/odometry/filtered')
    ]

    # 3. Composed Mode: Single container process with all navigation nodes loaded into it
    # Matches official Nav2 bringup architecture to eliminate DDS service discovery race conditions
    composition_group = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            Node(
                package='rclcpp_components',
                executable='component_container_isolated',
                name=container_name,
                output='screen',
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time, 'autostart': autostart}
                ]
            ),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[params_file, {'use_sim_time': use_sim_time}],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[params_file, {'use_sim_time': use_sim_time}]
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[params_file, {'use_sim_time': use_sim_time}],
                        remappings=[('cmd_vel', '/cmd_vel')]
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[params_file, {'use_sim_time': use_sim_time}]
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[params_file, {'use_sim_time': use_sim_time}]
                    ),
                ]
            )
        ]
    )

    # 4. Standalone Mode: fallback to isolated processes when use_composition is false
    standalone_group = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                remappings=remappings
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel', '/cmd_vel')]
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
        ]
    )

    # 5. Lifecycle Manager for Navigation
    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': navigation_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        use_composition_arg,
        container_name_arg,
        composition_group,
        standalone_group,
        lifecycle_manager_navigation_node
    ])
