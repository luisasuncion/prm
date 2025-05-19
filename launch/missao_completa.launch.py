from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Ruta para los otros launch files
    pkg_share = FindPackageShare('prm').find('prm')

    # Launch de Gazebo + Mundo
    launch_inicia_simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'inicia_simulacao.launch.py')
        )
    )

    # Launch que carga o robô, RViz, bridges, etc.
    launch_carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'carrega_robo.launch.py')
        )
    )

    # Nodo de controle principal (FSM)
    controle_robo_node = Node(
        package='prm',
        executable='controle_robo',
        name='controle_robo',
        output='screen'
    )

    # Nodo de visão para detectar a bandeira
    detecta_bandeira_node = Node(
        package='prm',
        executable='detecta_bandeira',
        name='detecta_bandeira',
        output='screen'
    )

    return LaunchDescription([
        launch_inicia_simulacao,
        launch_carrega_robo,
        controle_robo_node,
        detecta_bandeira_node
    ])
