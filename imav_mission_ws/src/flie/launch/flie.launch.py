from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    window_detector = Node(
        package='flie',
        executable='gate_red',
        name='gate_red_window_detector',
        output='screen'
    )

    rqt_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='m1_vision_view',
        arguments=['/m1/vision/detections']
    )

    return LaunchDescription([
        window_detector,
        rqt_view
    ])


