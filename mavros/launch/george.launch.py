from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import serial.tools.list_ports

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vid', default_value='0x0403'),
        DeclareLaunchArgument('pid', default_value='0x6001'),
        DeclareLaunchArgument('usb_description', default_value="USB-RS232 Cable - USB-RS232 Cable"),
        DeclareLaunchArgument('baudrate', default_value='57600'),  # New argument for baud rate
        OpaqueFunction(function=lambda context: find_device_port(context)),
    ] + launch_nodes())

def find_device_port(context):
    vid = int(context.launch_configurations['vid'], 16)
    pid = int(context.launch_configurations['pid'], 16)
    device_description = context.launch_configurations['usb_description']
    baudrate = context.launch_configurations['baudrate']  # Get the baud rate from the context

    available_ports = list(serial.tools.list_ports.comports())
    for port in available_ports:
        if vid == port.vid and pid == port.pid and device_description == port.description:
            fcu_url = f"{port.device}:{baudrate}"
            context.launch_configurations['fcu_url'] = fcu_url
            break

def launch_nodes():
    # Function to setup nodes, using fcu_url from the context
    fcu_url = LaunchConfiguration('fcu_url', default='/dev/ttyUSB2:57600')
    gcs_url = LaunchConfiguration('gcs_url', default='')
    tgt_system = LaunchConfiguration('tgt_system', default='1')
    tgt_component = LaunchConfiguration('tgt_component', default='1')
    log_output = LaunchConfiguration('log_output', default='screen')
    fcu_protocol = LaunchConfiguration('fcu_protocol', default='v2.0')
    respawn_mavros = LaunchConfiguration('respawn_mavros', default='false')
    namespace = LaunchConfiguration('namespace', default='mavros')

    mavros_pkg_dir = get_package_share_directory('mavros')
    mavros_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([mavros_pkg_dir, '/launch/george_node.launch.py']),
        launch_arguments={
            'pluginlists_yaml': f"{mavros_pkg_dir}/launch/george_pluginlists.yaml",
            'config_yaml': f"{mavros_pkg_dir}/launch/george_config.yaml",
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
            'tgt_system': tgt_system,
            'tgt_component': tgt_component,
            'fcu_protocol': fcu_protocol,
            'respawn_mavros': respawn_mavros,
            'namespace': namespace
        }.items(),
    )

    # Add the mavros_service_node execution
    mavros_service_node = Node(
        package='mavros',
        executable='mavros_service_node',
        output='screen'
    )
    
    return [mavros_launch_file, mavros_service_node]
