from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Define se roda em modo simulação (VRX) ou hardware real'
    )

    sim = LaunchConfiguration('sim')

    fcu_url_expr = PythonExpression([
        "'udp://127.0.0.1:14550@' if '", sim, "' == 'true' else '/dev/ttyACM0:57600'"
    ])

    # Nó do MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='whiteboat',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'true' if '", sim, "' == 'true' else 'false'"]),
            'fcu_url': fcu_url_expr,
            'target_system_id': 1,
            'system_id': 255,
            'component_id': 240,
        }],
    )

    # Inclui o launch de câmera
    whiteboat_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whiteboat_cam'),
                'launch',
                'whiteboat_cam.launch.py'
            )
        ),
        launch_arguments={'sim': sim}.items(),
    )

    # Nó Ponte MAVROS -> VRX (roda apenas no modo simulação)
    from launch.conditions import IfCondition
    bridge_node = Node(
        package='whiteboat_core',
        executable='mavros_to_vrx_bridge.py',
        name='mavros_to_vrx_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(sim)
    )

    return LaunchDescription([
        sim_arg,
        mavros_node,
        whiteboat_cam_launch,
        bridge_node,
    ])
