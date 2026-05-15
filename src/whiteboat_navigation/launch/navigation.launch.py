from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Grava rosbag da missão. Desativado por padrão para reduzir I/O em hardware embarcado.'
    )

    # Inclui o launch principal (MAVROS + câmera)
    whiteboat_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whiteboat_core'),
                'launch',
                'whiteboat.launch.py'
            )
        )
    )

    # Gravação de bag com ros2 bag record (equivalente ao rosbag record do ROS 1)
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', os.path.join(
                get_package_share_directory('whiteboat_navigation'),
                'bags',
                'whiteboat_mission'
            ),
            '/whiteboat/state',
            '/whiteboat/global_position/global',
            '/whiteboat/imu/data',
            '/whiteboat/rc/in',
            '/whiteboat/tf',
            '/whiteboat/tf_static',
            '/whiteboat/diagnostics',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag')),
    )

    return LaunchDescription([
        record_bag_arg,
        whiteboat_core_launch,
        rosbag_record,
    ])
