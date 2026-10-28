import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('flie')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'flie_config.rviz')

    # Node 1
    node_hardware = Node(
        package='takeoff',
        executable='odom_to_tf',
        name='hw'
    )

    # Node 5: RViz2 
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        node_hardware,
        TimerAction(period=4.0, actions=[node_rviz]),
    ])
