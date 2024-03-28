
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('fcu_url', default_value=''),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('tgt_system', default_value=''),
        DeclareLaunchArgument('tgt_component', default_value=''),
        DeclareLaunchArgument('pluginlists_yaml', default_value=''),
        DeclareLaunchArgument('config_yaml', default_value=''),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='mavros'),

        # Node definition
        Node(
            package='mavros',
            executable='mavros_node',  # Adjust if the executable name is different
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'fcu_url': LaunchConfiguration('fcu_url')},
                {'gcs_url': LaunchConfiguration('gcs_url')},
                {'tgt_system': LaunchConfiguration('tgt_system')},
                {'tgt_component': LaunchConfiguration('tgt_component')},
                {'fcu_protocol': LaunchConfiguration('fcu_protocol')},
                {'pluginlists_yaml': LaunchConfiguration('pluginlists_yaml')},
                {'config_yaml': LaunchConfiguration('config_yaml')},
            ],
            respawn=LaunchConfiguration('respawn_mavros'),
        ),
    ])
