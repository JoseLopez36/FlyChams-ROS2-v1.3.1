import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    output = DeclareLaunchArgument(
        "output",
        default_value='screen')

    publish_clock = DeclareLaunchArgument(
        "publish_clock",
        default_value='True')

    is_vulkan = DeclareLaunchArgument(
        "is_vulkan",
        default_value='True')

    host_ip = DeclareLaunchArgument(
        "host_ip",
        default_value='host.docker.internal')

    host_port = DeclareLaunchArgument(
        "host_port",
        default_value='41451')
    
    enable_api_control = DeclareLaunchArgument(
        "enable_api_control",
        default_value='True')
    
    enable_world_plot = DeclareLaunchArgument(
        "enable_world_plot",
        default_value='True')
  
    airsim_node = Node(
            package='airsim_wrapper',
            executable='airsim_node',
            namespace='airsim',
            name='airsim_node',
            output=LaunchConfiguration('output'),
            parameters=[{
                'is_vulkan': LaunchConfiguration('is_vulkan'),
                'update_airsim_state_every_n_sec': 0.025,
                'update_sim_clock_every_n_sec': 0.001,
                'update_airsim_status_every_n_sec': 0.100,
                'publish_clock': LaunchConfiguration('publish_clock'),
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'enable_api_control': LaunchConfiguration('enable_api_control'),
                'enable_world_plot': LaunchConfiguration('enable_world_plot')
            }])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(output)
    ld.add_action(publish_clock)
    ld.add_action(is_vulkan)
    ld.add_action(host_ip)
    ld.add_action(host_port)
    ld.add_action(enable_api_control)
    ld.add_action(enable_world_plot)
    ld.add_action(airsim_node)

    return ld
