from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_rov',
            executable='talker.py',
            name='talker',
            output='screen'
        ),
        Node(
            package='autonomous_rov',
            executable='listener.py',
            name='listener',
            output='screen'
        ),
        Node(
            package='autonomous_rov',
            executable='interface.py',
            name='interface',
            output='screen'
        ),
    ])
