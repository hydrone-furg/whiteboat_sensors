from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    # Nó do MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='whiteboat',
        output='screen',
        parameters=[{
            'fcu_url': '/dev/ttyACM0:57600',
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

    return LaunchDescription([
        sim_arg,
        mavros_node,
        whiteboat_cam_launch,
    ])
