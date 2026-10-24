import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    # 1. Declare Launch Arguments
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
        description='Full path to the map YAML file to load'
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

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the localization lifecycle stack'
    )
    autostart = LaunchConfiguration('autostart')

    # 2. Lifecycle nodes managed
    lifecycle_nodes = ['map_server', 'amcl']

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

    # 4. AMCL Localization Node
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

    # 5. Lifecycle Manager for Localization
    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        autostart_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization_node
    ])

