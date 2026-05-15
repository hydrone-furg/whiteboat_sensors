"""MAVROS (whiteboat) + square_node — garante stack antes do quadrado."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Mesmo significado que whiteboat.launch.py (true=VRX/SITL UDP)',
    )
    sim = LaunchConfiguration('sim')

    whiteboat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whiteboat_core'),
                'launch',
                'whiteboat.launch.py',
            )
        ),
        launch_arguments={'sim': sim}.items(),
    )

    # Configuração de uso de sim time dependendo do argumento "sim"
    use_sim_time_expr = PythonExpression(["'true' if '", sim, "' == 'true' else 'false'"])

    square_node = Node(
        package='whiteboat_navigation',
        executable='square.py',
        name='square_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_expr}]
    )

    # Dar tempo ao MAVROS ligar ao FCU antes do square_node subscrever/publicar.
    square_delayed = TimerAction(period=5.0, actions=[square_node])

    return LaunchDescription([sim_arg, whiteboat_launch, square_delayed])
