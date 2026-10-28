import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('flie')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'flie_config.rviz')

    # Node 1
    node_takeoff = Node(
        package='flie',
        executable='cf_takeoff',
        name='takeoff'
    )

    # Node 2
    node_translator = Node(
        package='flie',
        executable='cf_vel_translate',
        name='translator'
    )

    # Node 3
    node_tf = Node(
        package='flie',
        executable='tf_broadcaster_node',
        name='tf_broadcaster'
    )
    
    # Node 4
    node_waypoint = Node(
        package='flie',
        executable='waypoint_navigator',
        name='waypoint'
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
        node_takeoff,
        TimerAction(period=2.0, actions=[node_translator]),
        TimerAction(period=4.0, actions=[node_tf]),
        TimerAction(period=6.0, actions=[node_waypoint]),
        TimerAction(period=7.0, actions=[node_rviz]),
    ])
