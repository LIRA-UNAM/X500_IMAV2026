import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('takeoff')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'x500_config.rviz')
    
    #MicroXRCEAgent
    micro_ros_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Node 1
    node_hardware = Node(
        package='takeoff',
        executable='odom_to_tf',
        name='hw'
    )

    # Node 2: RViz2 
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        micro_ros_agent,
        TimerAction(period=7.0, actions=[node_hardware]),
        TimerAction(period=9.0, actions=[node_rviz]),
    ])
