from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.substitutions import FindPackageShare

import serial.tools.list_ports
import yaml

def find_device_port(context, *args, **kwargs):
    config_file_path = context.launch_configurations['config_file_path']
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)
    params = config['mavros']['ros__parameters']

    vid = params['vid']  # Convert hex to int
    pid = params['pid']  # Convert hex to int
    device_description = params['usb_description']
    baudrate = params['baud_rate']
    urls = []
    print(f"Searching for device with VID: {vid}, PID: {pid}, description: {device_description}")
    for port in serial.tools.list_ports.comports():
        print(f"current port: {port.device}, {port.vid}, {port.pid}, {port.description}")
        if port.vid == vid and port.pid == pid and device_description in port.description:
            url = port.device + ":" + str(baudrate)
            urls.append(url)
    # find device with the lowest port number
    if urls:
        url = sorted(urls)[0]
        print(f"Found device at: {url}")    
        return {'fcu_url': url}
    else:
        print("No device found.")
        return {'fcu_url': None}
    

def generate_launch_description():
    config_file_path_arg = DeclareLaunchArgument(
        'config_file_path',
        default_value='path/to/your/config.yaml',
        description='Path to the configuration file.'
    )

    def include_conditionally(context):
        fcu_url = find_device_port(context)['fcu_url']
        if fcu_url:
            return [IncludeLaunchDescription(
                AnyLaunchDescriptionSource([FindPackageShare('mavros'), '/launch/apm.launch']),
                launch_arguments={'fcu_url': fcu_url}.items(),
            )]
        else:
            return []
    
    return LaunchDescription([
        config_file_path_arg,
        OpaqueFunction(function=include_conditionally)
    ])
