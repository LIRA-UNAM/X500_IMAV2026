import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    micro_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        shell=False,
    )

    takeoff = Node(
        package='takeoff',
        executable='takeoff_node',
        name='takeoff_node',
        output='screen',
        emulate_tty=True,
    )

    simple = Node(
        package='takeoff',
        executable='simple_move',
        name='simple_move',
        output='screen',
        emulate_tty=True
    )

    translate = Node(
        package='takeoff',
        executable='translate_node',
        name='translate_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        micro_agent,
        takeoff,
        simple,
        translate
    ])