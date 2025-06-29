from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='prm',
            executable='controle_robo',
            name='controle_robo',
            output='screen'
        ),
        Node(
            package='prm',
            executable='detecta_bandeira',
            name='detecta_bandeira',
            output='screen'
        )
    ])
